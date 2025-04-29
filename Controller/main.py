import os

# Optional debugging: only activate debugpy if ENABLE_DEBUG is set
enable_debug = os.getenv("ENABLE_DEBUG", "").lower() in ("1", "true", "yes")
if enable_debug:
    try:
        import debugpy

        port = int(os.getenv("DEBUG_PORT", "5678"))
        debugpy.listen(("0.0.0.0", port))
        print(f"🐞 debugpy waiting for client on port {port}…")
        debugpy.wait_for_client()
    except ImportError:
        print("⚠️ debugpy not installed; continuing without debugger.")

# import matplotlib
from src.turtleBot.process import Process

# matplotlib.use("TkAgg")

if __name__ == "__main__":
    process = None
    try:
        # Read ROBOT_ID from environment, defaulting to 0
        robot_id = int(os.getenv("ROBOT_ID", "0"))
        process = Process("RobotProcess", robot_id, explore=True)

    except KeyboardInterrupt:
        print("Process interrupted by user (Ctrl+C). Stopping...")
        if process is not None:
            process.stop()
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        import traceback

        traceback.print_exc()
        if process is not None:
            process.stop()
