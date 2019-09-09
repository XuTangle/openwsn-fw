import socket
import struct

import time

import matplotlib.pyplot as plt
import numpy as np

# ================= definitions ===============================================

NUM_CELLS_ELAPSED       = 64
NUM_CELLS_USAGE_HIGH    = 48
NUM_CELLS_USAGE_LOW     = 16

SLOTDURATION            = 0.02 # second

EXP_DURATION            = 3 # in hour
mote_counter_asn_cellusage = {}


# open socket
socket_handler = socket.socket(socket.AF_INET6,socket.SOCK_DGRAM)
socket_handler.bind(('',61617))

while True:
    
    # wait for a request
    request,dist_addr = socket_handler.recvfrom(1024)
    
    hisAddress     = dist_addr[0]
    hisPort        = dist_addr[1]
    
    asnBytes       = struct.unpack('<HHB',request[-16:-11])
    tempBytes      = struct.unpack('<h',request[-9:-7])
    
    asn            = asnBytes[2] * 65536 * 65536 + asnBytes[1] * 65536 + asnBytes[0]
    temperature    = tempBytes[0]
    
    print 'temperature "{0}" at asn {1} from [{2}]:{3}'.format(temperature, asnBytes, hisAddress,hisPort)