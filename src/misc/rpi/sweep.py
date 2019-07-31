from sweeppy import Sweep

with Sweep('/dev/serial0') as sweep:
    sweep.start_scanning()

    for scan in sweep.get_scans():
        print('{}\n'.format(scan))
