import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/alphaone/Documents/safety-landing-point-detection-for-drone/install/gng_node'
