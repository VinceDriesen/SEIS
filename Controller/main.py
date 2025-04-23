import matplotlib
from src.turtleBot.process import Process

matplotlib.use("TkAgg")

process = Process("RobotProcess", 1)

if __name__ == "__main__":
    try:
        process.start()

    except KeyboardInterrupt:
        print("Process interrupted by user (Ctrl+C). Stopping...")
        process.stop()
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        import traceback
        traceback.print_exc()
        process.stop() 
