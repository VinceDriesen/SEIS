import matplotlib
from src.turtleBot.process import Process
from src.turtleBot.turtleBotStateMachine import TASK_EXPLORE, TASK_MOVE_TO

matplotlib.use("TkAgg")

if __name__ == "__main__":
    process = None
    try:
        process = Process("RobotProcess", 1)

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
