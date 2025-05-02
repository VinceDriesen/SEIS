"""
URL configuration for backend project.

The `urlpatterns` list routes URLs to views. For more information please see:
    https://docs.djangoproject.com/en/5.2/topics/http/urls/
Examples:
Function views
    1. Add an import:  from my_app import views
    2. Add a URL to urlpatterns:  path('', views.home, name='home')
Class-based views
    1. Add an import:  from other_app.views import Home
    2. Add a URL to urlpatterns:  path('', Home.as_view(), name='home')
Including another URLconf
    1. Import the include() function: from django.urls import include, path
    2. Add a URL to urlpatterns:  path('blog/', include('blog.urls'))
"""

import json
import time
from typing import Dict, Generator, List, Tuple
from django.contrib import admin
from django.http import StreamingHttpResponse
from django.urls import path
from ninja import NinjaAPI, Schema
from mqtt import MQTTManager, RobotPosition, RobotJob


class ErrorSchema(Schema):
    message: str


api = NinjaAPI()

manager = MQTTManager()
manager.start()


@api.post("/createJob", response={200: str, 403: ErrorSchema})
def createJob(request, x: float, y: float, robot_id: int):
    job_id = manager.create_job(x, y, str(robot_id))
    if job_id is not None:
        return 200, job_id
    else:
        return 403, {"message": "Job could not be created"}


@api.get("/isJobCompleted")
def isJobCompleted(request, job_id: str):
    return manager.get_job_status(job_id)


@api.get("/getRobotPos", response={200: RobotPosition, 403: ErrorSchema})
def getRobotPos(request, robot_id: int):
    pos = manager.get_position(str(robot_id))
    if pos is not None:
        return 200, pos
    else:
        return 403, {"message": "Could not retrieve robot position"}


@api.get("/robotStream", auth=None)
def robot_stream(request) -> StreamingHttpResponse:
    """
    SSE endpoint that streams an array of up to three robot positions,
    calling manager.get_position('0'), ('1'), and ('2') each iteration.
    """
    def event_generator() -> Generator[bytes, None, None]:
        robot_ids = ["0", "1", "2"]

        # Immediately send whatever we have at startup
        initial_positions = _gather_positions(robot_ids)
        yield f"data: {json.dumps(initial_positions)}\n\n".encode("utf-8")

        last_ping = time.time()
        while True:
            # send keep‑alive every 15s so intermediate proxies don't drop us
            if time.time() - last_ping > 15:
                yield b": ping\n\n"
                last_ping = time.time()

            # fetch the latest positions
            positions = _gather_positions(robot_ids)
            yield f"data: {json.dumps(positions)}\n\n".encode("utf-8")

            # throttle updates (adjust as desired)
            time.sleep(1)

    return StreamingHttpResponse(
        event_generator(),
        content_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
        },
    )


def _gather_positions(robot_ids: List[str]) -> List[Dict]:
    """
    Calls manager.get_position for each robot_id, and returns a list
    of position dicts for those robots that are currently known.
    """
    results = []
    for rid in robot_ids:
        pos = manager.get_position(rid)
        if pos:
            # Assuming pos has attributes .x and .y
            results.append({
                "robot_id": int(rid),
                "x": pos.x,
                "y": pos.y
            })
    return results

@api.get("/getRobotStatus")
def getRobotStatus(request, robot_id: int):
    pass


@api.get("/getRobotJobList", response={200: Tuple[List[RobotJob], List[RobotJob]], 403: ErrorSchema})
def getRobotJobList(request, robot_id: int):
    pending, completed = manager.get_robot_job_status(str(robot_id))
    if pending is not None and completed is not None:
        return 200, (pending, completed)
    else:
        return 403, {"message": "Could not yet job list from robot"}
    
    
@api.get("/occupancy-map", response={200: str, 404: ErrorSchema})
def getOccupancyMap(request):
    image = manager.get_occupancy_map()
    if image:
        return 200, image
    else:
        return 404, {"message": "No occupancy map image available"}



@api.get("/getRobots", response=List[int])
def getRobots(request):
    ids = manager.get_all_robots()
    return [int(id) for id in ids]

urlpatterns = [path("admin/", admin.site.urls), path("api/", api.urls)]
