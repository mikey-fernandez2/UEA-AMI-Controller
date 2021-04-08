# Mikey Fernandez, 03/19/2021
# Testing using pipes for inter process communication

# from signal import signal, SIGINT
import os, sys, time

# global path

# def handler(signal_received, frame):
#     # Handle any cleanup here
#     print("\nExiting\n")
#     os.remove(path)
#     exit(0)

def main():
    # Path to be created
    path = "/tmp/myfifo"

    try:
        os.mkfifo(path)

    except OSError as e:
        print("Failed to create FIFO: %s" % e)

    else:
        try:
            print("Sending...")
            i = 0
            while(1):
                fifo = open(path, 'w')
                fifo.write(str(i))
                fifo.close()

                i += 1
                time.sleep(1)


        except KeyboardInterrupt:
            print("\nTest send complete\n")
            os.remove(path)

    return

if __name__ == "__main__":
     # Tell Python to run the handler() function when SIGINT is recieved
    # signal(SIGINT, handler)

    main()