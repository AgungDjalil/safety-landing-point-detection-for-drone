import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mobi/Projects/drone-vajraakhasa/install/gng_node'
