import matplotlib
from src.turtleBot.process import Process
import debugpy

matplotlib.use("TkAgg")

if __name__ == "__main__":
    debugpy.listen(5678)
    print('debugpy waiting for client on port 5678…')
    debugpy.wait_for_client()
    
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
