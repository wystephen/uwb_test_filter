# Created by steve on 17-9-22 下午2:53
'''
                   _ooOoo_ 
                  o8888888o 
                  88" . "88 
                  (| -_- |) 
                  O\  =  /O 
               ____/`---'\____ 
             .'  \\|     |//  `. 
            /  \\|||  :  |||//  \ 
           /  _||||| -:- |||||-  \ 
           |   | \\\  -  /// |   | 
           | \_|  ''\---/''  |   | 
           \  .-\__  `-`  ___/-. / 
         ___`. .'  /--.--\  `. . __ 
      ."" '<  `.___\_<|>_/___.'  >'"". 
     | | :  `- \`.;`\ _ /`;.`/ - ` : | | 
     \  \ `-.   \_ __\ /__ _/   .-` /  / 
======`-.____`-.___\_____/___.-`____.-'====== 
                   `=---=' 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
         佛祖保佑       永无BUG 
'''


import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    src = np.loadtxt("test_data.txt")
    out = np.loadtxt("out_data.txt",delimiter=',')

    plt.figure()
    plt.plot(src[:,0],src[:,1],'r-+',label = "source data")
    plt.plot(out[:,0],out[:,1],'b-+',label = "out data")
    plt.grid()
    plt.legend()
    plt.show()