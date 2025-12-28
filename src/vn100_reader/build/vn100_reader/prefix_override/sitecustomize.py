import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pratham/Documents/vn100_reader/install/vn100_reader'
