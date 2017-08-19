import time
import signal

def test(i):
    time.sleep(i%4)
    print("%d within time"%(i))
    return i


def handler(signum, frame):
    raise AssertionError
if __name__ == '__main__':


    i = 0
    for i in range(1,10):
        try:
            signal.signal(signal.SIGALRM, handler)
            signal.alarm(3)
            s=2
            time.sleep(i % 4)
            print("%d within time" % (i))

            signal.alarm(0)
        except :
            print("%d timeout"%(i))