"""
A simple script to spawn two processes, running parallel, and writing
to the n7d0 device
"""

from multiprocessing import Process

def open_and_write(device='/dev/n7d0', msg='11111'):
    fo = open(device, "w")
    print("Successfully opened device:", fo.name)
    for i in range(100000):
        fo.write(msg)
        fo.flush()
    fo.close()

if __name__ == "__main__":
    device = '/dev/n7d0'
    msgs = ['11111', '22222']
    procs = []

    # instantiating process with arguments
    for msg in msgs:
        proc = Process(target=open_and_write, args=(device, msg,))
        procs.append(proc)
        proc.start()

    # complete the processes
    for proc in procs:
        proc.join()
