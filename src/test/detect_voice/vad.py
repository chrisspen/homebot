import scipy.fftpack as sf
import numpy as np

def hasHumanVoice(X, threshold, F_sample, Low_cutoff=50, High_cutoff= 300):
        """ Searching presence of frequencies on a real signal using FFT
        Inputs
        =======
        X: 1-D numpy array, the real time domain audio signal (single channel time series)
        Low_cutoff: float, frequency components below this frequency will not pass the filter (physical frequency in unit of Hz)
        High_cutoff: float, frequency components above this frequency will not pass the filter (physical frequency in unit of Hz)
        F_sample: float, the sampling frequency of the signal (physical frequency in unit of Hz)
        threshold: Has to be standardized once to say how much power must be there in real vocal signal frequencies.    
        """        

        M = X.size # let M be the length of the time series
        Spectrum = sf.rfft(X, n=M) 
        [Low_cutoff, High_cutoff, F_sample] = map(float, [Low_cutoff, High_cutoff, F_sample])

        #Convert cutoff frequencies into points on spectrum
        [Low_point, High_point] = map(lambda F: F/F_sample * M, [Low_cutoff, High_cutoff])

        totalPower = np.sum(Spectrum)
        fractionPowerInSignal = np.sum(Spectrum[Low_point : High_point])/totalPower # Calculating fraction of power in these frequencies

        if fractionPowerInSignal > threshold:
            return 1
        else:
            return 0

voiceVector = []
for window in fullAudio: # Run a window of appropriate length across the audio file
    voiceVector.append(hasHumanVoice(window, threshold, samplingRate)
