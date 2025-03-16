import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dtc/airlab_ws/varunkasi_ws/install/thermal_calibration_rqt'
