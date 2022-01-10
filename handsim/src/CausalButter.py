# "CausalButter.py" is the python class I wrote to implement the causal butterworth filter (same algorithm as Tianhe_filter but in python)

# reference here: http://www.exstrom.com/journal/sigproc/
#                 http://www.exstrom.com/journal/sigproc/bwbpf.c
import math

class CausalButter:
    # the default init method assumes Causal butter is a bandpass filter, and allows signal frequency from f_low to f_high to pass.
    # when bandstop == 1, the causal butter filter becomes bandstop and signal frequency from f_low to f_high will be filtered out
    def __init__(self,order,f_low,f_high,sampleRate,bandstop=0):
        if order%4 != 0:
            print ("the order of CausalButter has to be a multiple of 4")
            return 
        self.f_high = f_high
        self.f_low = f_low
        self.sampleRate = sampleRate
        self.order = order
        self.bandstop = bandstop

        s = self.sampleRate
        a = math.cos(math.pi*(f_high+f_low)/s)/math.cos(math.pi*(f_high-f_low)/s)
        a2 = a**2
        b = math.tan(math.pi*(f_high-f_low)/s);
        b2 = b**2
        n = int(order/4)
        self.n = int(order/4)
        self.a = a
        self.a2 = a2
        self.b = b
        self.b2 = b2

        self.A = [0]*n
        self.d1 = [0]*n
        self.d2 = [0]*n
        self.d3 = [0]*n
        self.d4 = [0]*n
        self.w0 = [0]*n
        self.w1 = [0]*n
        self.w2 = [0]*n
        self.w3 = [0]*n
        self.w4 = [0]*n
        
        if self.bandstop ==0:
            for i in range(n):
                r = math.sin(math.pi*(2.0*i+1.0)/(4.0*n));
                s = b2 + 2.0*b*r + 1.0;
                self.A[i] = b2/s;
                self.d1[i] = 4.0*a*(1.0+b*r)/s;
                self.d2[i] = 2.0*(b2-2.0*a2-1.0)/s;
                self.d3[i] = 4.0*a*(1.0-b*r)/s;
                self.d4[i] = -(b2 - 2.0*b*r + 1.0)/s;
        else:
            for i in range(n):
                r = math.sin(math.pi*(2.0*i+1.0)/(4.0*n));
                s = b2 + 2.0*b*r + 1.0;
                self.A[i] = 1/s;
                self.d1[i] = 4.0*a*(1.0+b*r)/s;
                self.d2[i] = 2.0*(b2-2.0*a2-1.0)/s;
                self.d3[i] = 4.0*a*(1.0-b*r)/s;
                self.d4[i] = -(b2 - 2.0*b*r + 1.0)/s;
            self.r = 4.0*a;
            self.s = 4.0*a2+2.0;

    def inputData(self, raw_data):
        #BUGMAN 5/24/2017 modified the npts
        npts = len(raw_data);
        filtered_data = [None]*npts;
       
        # the default is to create a bandpass causal butter filter
        if self.bandstop ==0:
            for pnt in range(npts):
                x = raw_data[pnt]
                for i in range(self.n):
                    self.w0[i] = self.d1[i]*self.w1[i] + self.d2[i]*self.w2[i] + self.d3[i]*self.w3[i] + self.d4[i]*self.w4[i] + x;
                    x = self.A[i]*(self.w0[i] - 2.0*self.w2[i] + self.w4[i]);
                    self.w4[i] = self.w3[i];
                    self.w3[i] = self.w2[i];
                    self.w2[i] = self.w1[i];
                    self.w1[i] = self.w0[i];
                filtered_data[pnt] = x
        else:
            for pnt in range(npts):
                x = raw_data[pnt]
                for i in range(self.n):
                    self.w0[i] = self.d1[i]*self.w1[i] + self.d2[i]*self.w2[i] + self.d3[i]*self.w3[i] + self.d4[i]*self.w4[i] + x;
                    # bandstop method changed some coefficients here
                    x = self.A[i]*(self.w0[i] - self.r*self.w1[i] + self.s*self.w2[i] - self.r*self.w3[i]+ self.w4[i]);
                    self.w4[i] = self.w3[i];
                    self.w3[i] = self.w2[i];
                    self.w2[i] = self.w1[i];
                    self.w1[i] = self.w0[i];
                filtered_data[pnt] = x


        return filtered_data


    def printParams(self):
        print('self.f_high is ', self.f_high)
        print('self.f_low is ', self.f_low)
        print('self.sampleRate is ', self.sampleRate)
        print('self.order is ', self.order)
        print('self.n is ', self.n)
        print('self.a is ', self.a)
        print('self.a2 is ', self.a2)
        print('self.b is ', self.b)
        print('self.b2 is ', self.b2)
        print('self.A is ', self.A)
        print('self.d1 is ', self.d1)
        print('self.d2 is ', self.d2)
        print('self.d3 is ', self.d3)
        print('self.d4 is ', self.d4)
        print('self.w0 is ', self.w0)
        print('self.w1 is ', self.w1)
        print('self.w2 is ', self.w2)
        print('self.w3 is ', self.w3)
        print('self.w4 is ', self.w4)
        