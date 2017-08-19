
from time import ctime, sleep
from threading import Thread

class test:
    name=[1,2,3,4]
    def loop0(self):
        print('start loop 0 at :',ctime())
        for i in range(len(self.name)):
            self.name[i]+=1
            sleep(1)
        print('end loop 0 at :',ctime())

def loop1():
    print('start loop 1 at:',ctime())
    sleep(2)
    print('loop 1 done at:',ctime())

class myThread (Thread):
    def __init__(self, name):
        Thread.__init__(self)
        self.name=name
    def run(self):
        for i in range (2):
            sleep(2)
            print('%s:un%i'%(self.name,i))



def main():
    print('starting at:',ctime())
    testlist=[]
    for i in range(5):
        testlist.append(test())
    thread=[]
    for i in range(5):
        t1=Thread(target=testlist[i].loop0)
        t1.start()
        thread.append(t1)
    for i in range(5):
        thread[i].join()
    print('all done at:',ctime())

if __name__=='__main__':
    main()

    # Threads=[]
    # for i in range(5):
    #     t=myThread(i)
    #     Threads.append(t)
    # for i in range(5):
    #     Threads[i].start()
    # for i in range(5):
    #     Threads[i].join()
    # print('end')



    # main()