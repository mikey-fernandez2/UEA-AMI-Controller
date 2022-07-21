# "BesselFilter.py implements the Bessel filters

from scipy import signal
import numpy as np

class BesselFilterArr():
    # this creates an array of Bessel filters for processing multiple signals
    def __init__(self, numChannels, order, critFreqs, fs, filtType):
        self.numChannels = numChannels

        self.filters = {'sos': [], 'zi': []}
        for _ in range(self.numChannels):
            filter_sos = signal.bessel(N=order, Wn=critFreqs, btype=filtType, output='sos', fs=fs, analog=False)
            filter_zi = signal.sosfilt_zi(filter_sos)

            self.filters['sos'].append(filter_sos)
            self.filters['zi'].append(filter_zi)


    def filter(self, sig):
        filterOut = np.zeros_like(sig)

        for i in range(self.numChannels):
            out, self.filters['zi'][i] = signal.sosfilt(self.filters['sos'][i], sig[i, :], zi=self.filters['zi'][i])
            filterOut[i, :] = out

        return filterOut

    def getFilter(self, channelNum):
        return self.filters['sos'][channelNum], self.filters['zi'][channelNum]

    def printFilters(self):
        for i in range(self.numChannels):
            print(f'Channel {i}: {self.getFilter(i)}')
