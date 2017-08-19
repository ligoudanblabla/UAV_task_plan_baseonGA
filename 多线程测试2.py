# -*- coding: utf-8 -*-
# @Time    : 2017/7/27 23:19
# @Author  : BaWang
# @Site    : 
# @Summart : 介绍文档的主要功能
# -*- coding: utf-8 -*-
import threading
import time


class Test:
    def __init__(self):
        # threading.Thread.__init__(self)
        self._sName = "machao"

    def process(self):
        # args是关键字参数，需要加上名字，写成args=(self,)
        th1 = threading.Thread(target=Test.buildList, args=(self,))
        th1.start()
        th1.join()

    def buildList(self):
        print("start")
        time.sleep(3)


test = Test()
test.process()

test2 = Test()
test2.process()