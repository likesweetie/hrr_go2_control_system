import lcm
from gps_lcm_type import gps_t

def my_handler(channel, data):
    msg = gps_t.decode(data)
    print("Received message on channel \"%s\"" % channel)
    print("   timestamp   = %s" % str(msg.timestamp))

lc = lcm.LCM()
subscription = lc.subscribe("GPS_DATA", my_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass
