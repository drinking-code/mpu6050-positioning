import numpy as np


def tons_lowpass_filter(cutoff_freq, sample_time, x0, x1, x2, y1, y2):
    # https://gist.github.com/moorepants/bfea1dc3d1d90bdad2b5623b4a9e9bee
    """
    :param cutoff_freq: Cutoff frequency
    :param sample_time: Time delta to last sample
    :param x0: Current sample (unfiltered)
    :param x1: Last sample (unfiltered)
    :param x2: Second last sample (unfiltered)
    :param y1: Last sample (filtered)
    :param y2: Second last sample (filtered)
    :return: Current sample (filtered)
    """

    # Compute coefficients of the state equation
    a = (2 * np.pi * cutoff_freq) ** 2
    b = np.sqrt(2) * 2 * np.pi * cutoff_freq

    # Integrate the filter state equation using the midpoint Euler method with step h
    h = sample_time
    denom = 4 + 2 * h * b + h ** 2 * a

    A = (4 + 2 * h * b - h ** 2 * a) / denom
    B = 4 * h / denom
    C = -4 * h * a / denom
    D = (4 - 2 * h * b - h ** 2 * a) / denom
    E = 2 * h ** 2 * a / denom
    F = 4 * h * a / denom

    y = (A + B * D / h) * y1 + (B * C - B * D / h) * y2 + E / 2 * x0 + (B * F / 2 + E / 2) * x1 + B * F / 2 * x2

    return y


last_sample = None
second_last_sample = None
last_sample_filtered = [0, 0, 0]
second_last_sample_filtered = [0, 0, 0]


def butter_signal(cutoff_freq, sample_time, sample):
    """
    :param cutoff_freq: Cutoff frequency
    :param sample_time: Time delta to last sample
    :param sample: Sample (unfiltered)
    :return: Sample (filtered)
    """
    global last_sample
    global second_last_sample
    global last_sample_filtered
    global second_last_sample_filtered

    if second_last_sample is not None:
        current_sample_filtered = [
            tons_lowpass_filter(
                cutoff_freq, sample_time,
                current_sample, last_sample[i], second_last_sample[i],
                last_sample_filtered[i], second_last_sample_filtered[i]
            ) for i, current_sample in enumerate(sample)
        ]
    else:
        current_sample_filtered = [0, 0, 0]

    second_last_sample_filtered = last_sample_filtered
    last_sample_filtered = current_sample_filtered

    second_last_sample = last_sample
    last_sample = sample

    return current_sample_filtered
