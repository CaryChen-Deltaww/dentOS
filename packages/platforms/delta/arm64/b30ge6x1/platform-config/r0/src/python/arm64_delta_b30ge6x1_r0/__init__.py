#!/usr/bin/python

from onl.platform.base import *
from onl.platform.delta import *

class OnlPlatform_arm64_delta_b30ge6x1_r0(OnlPlatformDelta,
                                       OnlPlatformPortConfig_24x1_6x10):
    PLATFORM='arm64-delta-b30ge6x1-r0'
    MODEL="B30GE6X1"
    SYS_OBJECT_ID=".30.6"

    def baseconfig(self):

        # Insert platform drivers
        self.insmod("arm64-delta-b30ge6x1-cpld.ko")

        ########### initialize I2C bus ###########
        self.new_i2c_devices(
            [
                # inititate LM75
                ('lm75', 0x4a, 10),
                ('lm75', 0x4b, 10),
                ('lm75', 0x4f, 10),
                # FAN Controller
                ('adt7473', 0x2e, 11),
            ]
        )

        # Insert prestera kernel module
        self.modprobe('mvMbusDrv')
        self.modprobe('mvIntDrv')
        self.modprobe('mvDmaDrv')
        self.modprobe('prestera_shm')
        subprocess.call('mkdir /etc/mvsw', shell=True)
        subprocess.call('cp /lib/platform-config/%s/onl/mvsw_platform.xml /etc/mvsw/' % self.PLATFORM, shell=True)
        subprocess.call('cp /lib/platform-config/%s/onl/sw-agentd.init /etc/init.d/sw-agentd' % self.PLATFORM, shell=True)
        subprocess.call('chmod 0777 /etc/init.d/sw-agentd', shell=True)
        subprocess.call('update-rc.d sw-agentd defaults', shell=True)
        # set up systemctl rules
        #for swp in range(1, 27):
        #    cmd = "systemctl enable switchdev-online@swp%d" % swp
        #    subprocess.check_call(cmd, shell=True)

        return True
