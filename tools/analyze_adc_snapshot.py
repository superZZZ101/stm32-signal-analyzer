import cmath
import math
import re
import struct
import sys
from pathlib import Path


FS = 4_212_000.0
DECIMATION = 4
ROOT = Path(__file__).resolve().parents[1]


def fft(values):
    values = list(values)
    count = len(values)
    position = 0
    for index in range(1, count):
        bit = count >> 1
        while position & bit:
            position ^= bit
            bit >>= 1
        position ^= bit
        if index < position:
            values[index], values[position] = values[position], values[index]

    length = 2
    while length <= count:
        root = cmath.exp(-2j * math.pi / length)
        half = length >> 1
        for start in range(0, count, length):
            factor = 1.0 + 0.0j
            for offset in range(half):
                even = values[start + offset]
                odd = values[start + offset + half] * factor
                values[start + offset] = even + odd
                values[start + offset + half] = even - odd
                factor *= root
        length <<= 1
    return values


def read_samples(path):
    data = Path(path).read_bytes()
    return list(struct.unpack("<%dH" % (len(data) // 2), data))


def load_filter_taps():
    source = (ROOT / "Core/Src/measurement_tasks.c").read_text(encoding="utf-8")
    block = re.search(
        r"measurement_filter_taps\[.*?\]\s*=\s*\{(.*?)\};",
        source,
        flags=re.S,
    ).group(1)
    return [float(value) for value in re.findall(r"[-+]?\d+\.\d+(?:e[-+]?\d+)?", block, re.I)]


def peak_report(signal, sample_rate, lower, upper, limit=8):
    count = len(signal)
    mean = sum(signal) / count
    window = [0.5 - 0.5 * math.cos(2.0 * math.pi * n / (count - 1)) for n in range(count)]
    window_sum = sum(window)
    spectrum = fft((signal[n] - mean) * window[n] for n in range(count))
    first = max(1, int(lower * count / sample_rate))
    last = min(count // 2 - 1, int(upper * count / sample_rate) + 1)
    candidates = []
    for bin_index in range(first, last + 1):
        magnitude = abs(spectrum[bin_index])
        if magnitude >= abs(spectrum[bin_index - 1]) and magnitude >= abs(spectrum[bin_index + 1]):
            candidates.append((magnitude, bin_index))
    candidates.sort(reverse=True)
    result = []
    for magnitude, bin_index in candidates[:limit]:
        result.append(
            (
                bin_index * sample_rate / count,
                2.0 * magnitude / window_sum,
                bin_index,
            )
        )
    return result


def filter_and_decimate(samples, taps):
    mean = sum(samples) / len(samples)
    mask = len(samples) - 1
    half = (len(taps) - 1) // 2
    output = []
    for output_index in range(4096):
        centre = output_index * DECIMATION
        value = 0.0
        for tap_index, coefficient in enumerate(taps):
            sample_index = (centre + tap_index + len(samples) - half) & mask
            value += coefficient * (samples[sample_index] - mean)
        output.append(value)
    return output


def tone_amplitude(samples, frequency):
    mean = sum(samples) / len(samples)
    accumulator = 0.0j
    for index, sample in enumerate(samples):
        accumulator += (sample - mean) * cmath.exp(-2j * math.pi * frequency * index / FS)
    return 2.0 * abs(accumulator) / len(samples)


def channel_tone_amplitude(samples, phase, frequency):
    channel = samples[phase::3]
    mean = sum(channel) / len(channel)
    accumulator = 0.0j
    for index, sample in enumerate(channel):
        raw_index = phase + 3 * index
        accumulator += (sample - mean) * cmath.exp(-2j * math.pi * frequency * raw_index / FS)
    return mean, 2.0 * abs(accumulator) / len(channel)


def crossing_frequency(samples):
    mean = sum(samples) / len(samples)
    crossings = []
    previous = samples[0] - mean
    for index in range(1, len(samples)):
        current = samples[index] - mean
        if previous < 0.0 <= current and current != previous:
            crossings.append((index - 1) + (-previous / (current - previous)))
        previous = current
    periods = [crossings[index] - crossings[index - 1] for index in range(1, len(crossings))]
    average_period = sum(periods) / len(periods)
    return FS / average_period, min(periods), max(periods)


def segment_report(samples, frequency):
    print("1024-sample segments:")
    for start in range(0, len(samples), 1024):
        segment = samples[start : start + 1024]
        mean = sum(segment) / len(segment)
        rms = math.sqrt(sum((sample - mean) ** 2 for sample in segment) / len(segment))
        accumulator = 0.0j
        for offset, sample in enumerate(segment):
            raw_index = start + offset
            accumulator += (sample - mean) * cmath.exp(
                -2j * math.pi * frequency * raw_index / FS
            )
        amplitude = 2.0 * abs(accumulator) / len(segment)
        local_frequency, local_minimum_period, local_maximum_period = crossing_frequency(segment)
        phase_degrees = math.degrees(cmath.phase(accumulator))
        print(
            f"  {start:5d}: vpp={max(segment)-min(segment):4d} "
            f"rms={rms:7.2f} tone_amp={amplitude:7.2f} "
            f"f={local_frequency:9.1f} phase={phase_degrees:7.1f}"
        )


def analyze(path, taps):
    samples = read_samples(path)
    mean = sum(samples) / len(samples)
    rms = math.sqrt(sum((value - mean) ** 2 for value in samples) / len(samples))
    print(f"\n{Path(path).name}")
    print(f"raw: min={min(samples)} max={max(samples)} vpp={max(samples)-min(samples)} mean={mean:.2f} rms={rms:.2f}")
    raw_peaks = peak_report(samples, FS, 10_000.0, FS / 2.0, 8)
    print("raw FFT peaks:")
    for frequency, amplitude, bin_index in raw_peaks:
        print(f"  bin={bin_index:5d} f={frequency:10.1f} Hz amplitude={amplitude:8.2f} code")

    filtered = filter_and_decimate(samples, taps)
    filtered_peaks = peak_report(filtered, FS / DECIMATION, 10_000.0, 526_000.0, 8)
    print("filtered /4 FFT peaks:")
    for frequency, amplitude, bin_index in filtered_peaks:
        print(f"  bin={bin_index:5d} f={frequency:10.1f} Hz amplitude={amplitude:8.2f} code")

    strongest_frequency = raw_peaks[0][0]
    crossing_hz, minimum_period, maximum_period = crossing_frequency(samples)
    print(
        f"crossings: f={crossing_hz:.2f} Hz, period range="
        f"{minimum_period:.3f}..{maximum_period:.3f} samples"
    )
    print(f"raw DFT amplitude at {strongest_frequency:.1f} Hz: {tone_amplitude(samples, strongest_frequency):.2f}")
    for phase in range(3):
        channel_mean, amplitude = channel_tone_amplitude(samples, phase, strongest_frequency)
        print(f"ADC phase {phase}: mean={channel_mean:.2f} amplitude={amplitude:.2f} code")
    segment_report(samples, crossing_hz)
    print("first 24 codes:", " ".join(str(value) for value in samples[:24]))


def main():
    taps = load_filter_taps()
    print(f"filter taps: {len(taps)}, sum={sum(taps):.9f}")
    for argument in sys.argv[1:]:
        analyze(argument, taps)


if __name__ == "__main__":
    main()
