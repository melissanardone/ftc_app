package org.steelhead.ftc;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cDevice;

import java.util.concurrent.locks.Lock;

/**
 * Created by alecmatthews on 10/18/16.
 */

public class Adafruit_ColorSensor implements I2cController.I2cPortReadyCallback {
    String LogId = "AdafrutCS:";

    static final byte
        ADDRESS         = 0x29, //Device Address

        ENABLE          = 0x00 - 128, //Enable Register
        ENABLE_AEIN     = 0x10,
        ENABLE_WEN      = 0x08,
        ENABLE_AEN      = 0x02,
        ENABLE_PON      = 0x01,

        ATIME           = 0x01 - 128,

        WTIME           = 0x03 - 128,

        AILTL           = 0x04 - 128,
        AILTH           = 0x05 - 128,
        AIHTL           = 0x06 - 128,
        AIHTH           = 0x07 - 128,

        PERS            = 0x0C - 128,

        CONFIG          = 0x0D - 128,
        CONFIG_WLONG    = 0x02,

        CONTROL         = 0x0F - 128,
        CONTROL_AGAIN1  = 0x00,
        CONTROL_AGAIN4  = 0x01,
        CONTROL_AGAIN16 = 0x10,
        CONTROL_AGAIN60 = 0x11,

        ID              = 0x12 - 128,
        STATUS          = 0x13 - 128,
        STATUS_AINT     = 0x10,
        STATUS_AVALID   = 0x01,

        DATA_START      = 0x14 - 128,   //Color Data Start
        DATA_LENGTH     = 0x08,         //Color Data Length

        CDATA           = 0x14 - 128,
        CDATAH          = 0x15 - 128,
        RDATA           = 0x16 - 128,
        RDATAH          = 0x17 - 128,
        GDATA           = 0x18 - 128,
        GDATAH          = 0x19 - 128,
        BDATA           = 0x1A - 128,
        BDATAH          = 0x1B - 128;

    static final byte
        READ_MODE = 0x00 - 128,
        WRITE_MODE = 0x00;

    static final int
        CACHE_MODE = 0,
        DEV_ADDR = 1,
        REG_NUMBER = 2,
        REG_COUNT = 3,
        DATA_OFFSET = 4,
        ACTION_FLAG = 31;   //0 = idle, -1 = transfer in progress

    private ArrayQueue<I2cTransfer> transferQueue;
    private I2cDevice           csDev;
    private byte                csDevAddr;
    private byte[]              rCache;
    private byte[]              wCache;
    private Lock                rLock;
    private Lock                wLock;

    private int clear           = 0;
    private int red             = 0;
    private int green           = 0;
    private int blue            = 0;
    private int aTimeValue      = 0;
    private int controlValue    = 0;
    private int idValue         = 0;

    public Adafruit_ColorSensor(HardwareMap hardwareMap, String deviceName) {
        transferQueue = new ArrayQueue<I2cTransfer>();
        csDev = hardwareMap.i2cDevice.get(deviceName);
        csDevAddr = 2*ADDRESS;

        rCache = csDev.getI2cReadCache();
        wCache = csDev.getI2cWriteCache();
        rLock = csDev.getI2cReadCacheLock();
        wLock = csDev.getI2cWriteCacheLock();

        addWriteRequest(ENABLE, (byte) (ENABLE_PON | ENABLE_AEN));
        addReadRequest(ID, (byte) 1);
        executeCommands();
        csDev.registerForI2cPortReadyCallback(this);
        setGain(16);
        setIntegrationTime(50);
    }

    public void close() {
        transferQueue.close();
        csDev.deregisterForPortReadyCallback();
        csDev.close();
    }

    @Override
    public void portIsReady(int port) {
        try {
            rLock.lock();
            if (rCache[0] == wCache[0] && rCache[1] == wCache[1] && rCache[2] == wCache[2] &&
                    rCache[3] == wCache[3]) {
                rCache[DEV_ADDR] = 0;
                storeReceivedData();
                executeCommands();
            } else {
                csDev.readI2cCacheFromController();
            }
        } finally {
            rLock.unlock();
        }
    }

    private void readCommand(byte regNumber, byte regCount) {
        try {
            wLock.lock();
            wCache[CACHE_MODE] = READ_MODE;
            wCache[DEV_ADDR] = csDevAddr;
            wCache[REG_NUMBER] = regNumber;
            wCache[REG_COUNT] = regCount;
            wCache[ACTION_FLAG] = -1;
        } finally {
            wLock.unlock();
        }
        csDev.writeI2cCacheToController();
    }

    private void writeCommand(byte regNumber, byte regCount, long regValue, boolean isLowFirst) {
        try {
            wLock.lock();
            wCache[CACHE_MODE] = WRITE_MODE;
            wCache[DEV_ADDR] = csDevAddr;
            wCache[REG_NUMBER] = regNumber;
            wCache[REG_COUNT] = regCount;
            if (regCount == 1) {
                wCache[DATA_OFFSET] = (byte) (regValue & 0xFF);
            } else if (isLowFirst) {
                for (int i = 0; i < regCount; i++) {
                    wCache[DATA_OFFSET + i] = (byte) (regValue & 0xFF);
                    regValue >>= 8;
                }
            } else {
                for (int i = regCount - 1; i >= 0; i--) {
                    wCache[DATA_OFFSET + i] = (byte) (regValue & 0xFF);
                    regValue >>= 8;
                }
            }
            wCache[ACTION_FLAG] = -1;
        } finally {
            wLock.unlock();
        }
        csDev.writeI2cCacheToController();
    }

    private void executeCommands() {
        boolean isRead  = false;
        boolean isWrite = false;
        boolean isLowfirst = false;
        byte regNumber = 0;
        byte regCount = 0;
        long regValue = 0;
        try {
            wLock.lock();
            if (!transferQueue.isEmpty()) {
                I2cTransfer element = transferQueue.remove();
                isWrite = element.mode == WRITE_MODE;
                isRead = element.mode == READ_MODE;
                regNumber = element.regNumber;
                regCount = element.regCount;
                regValue = element.regValue;
                isLowfirst = element.isLowFirst;
                element = null;
            }
        } finally {
            wLock.unlock();
        }
        if (isWrite) {
            writeCommand(regNumber, regCount, regValue, isLowfirst);
        } else if (isRead) {
            readCommand(regNumber, regCount);
        } else {
            readCommand(DATA_START, DATA_LENGTH);
        }
    }

    private void addWriteRequest(byte regNumber, byte regValue) {
        addRequest(new I2cTransfer(regNumber, (byte) 1, regValue, true));
    }

    private void addWriteRequest(byte regNumber, byte regCount, byte regValue) {
        addRequest(new I2cTransfer(regNumber, regCount, regValue, true));
    }

    private void addWriteRequest(byte regNumber, byte regCount, byte regValue, boolean isLowFirst) {
        addRequest(new I2cTransfer(regNumber, regCount, regValue, isLowFirst));
    }

    private void addReadRequest(byte regNumber, byte regCount) {
        transferQueue.add(new I2cTransfer(regNumber, regCount));
    }

    private void addRequest(I2cTransfer element) {
        try {
            rLock.lock();
            try {
                wLock.lock();
                transferQueue.add(element);
            } finally {
                wLock.unlock();
            }
        } finally {
            rLock.unlock();
        }
    }

    private int getWord(int lowByte, int highByte) {
        int low = rCache[DATA_OFFSET + lowByte - DATA_START] & 0xFF;
        int high = rCache[DATA_OFFSET + highByte - DATA_START] & 0xFF;
        return 256*high + low;
    }

    private void storeReceivedData() {
        byte regNumber = rCache[REG_NUMBER];
        byte regCount = rCache[REG_COUNT];

        switch (regNumber) {
            case DATA_START:
                if(regCount == DATA_LENGTH) {
                    clear = getWord(CDATA, CDATAH);
                    red = getWord(RDATA, RDATAH);
                    green = getWord(GDATA, GDATAH);
                    blue = getWord(BDATA, BDATAH);
                }
                break;
            case ENABLE:
                break;
            case ATIME:
                aTimeValue = rCache[DATA_OFFSET] & 0xFF;
                break;
            case CONTROL:
                controlValue = rCache[DATA_OFFSET] & 0xFF;
                break;
            case ID:
                idValue = rCache[DATA_OFFSET] & 0xFF;
                break;
            default:
                Log.e(LogId, String.format("Unexpected R[0x%02x] = 0x%02x received", regNumber, rCache[DATA_OFFSET]));
                break;
        }
    }

    private int colorTemperature() {
        if (red + green + blue == 0) return 999;

        double r = red;
        double g = green;
        double b = blue;

        double x = (-0.14282 * r) + (1.54924 * g) + (-0.95641 * b);
        double y = (-0.32466 * r) + (1.57837 * g) + (-0.73191 * b);
        double z = (-0.68202 * r) + (0.77073 * g) + (0.56332 * b);

        double xyz = x + y + z;

        double xC = x/xyz;
        double yC = y/xyz;

        double n = (xC - 0.3320) / (0.1858 - yC);
        double CCT = 449.0 *n*n*n + 3525.0 *n*n + 6823.3 * n + 5520.33;

        if (CCT > 29999) CCT = 29999;
        if (CCT < 1000) CCT = 1000;
        return (int) CCT;
    }

    public int clearColor() {
        return clear;
    }
    public int redColor() {
        return red;
    }
    public int greenColor() {
        return green;
    }
    public int blueColor() {
        return blue;
    }
    public int colorTemp() {
        return colorTemperature();
    }

    public boolean isIdOk() {
        return (idValue == 0x44 || idValue == 0x4D);
    }

    public void setIntegrationTime(double milliSeconds) {
        int count = (int) (milliSeconds/2.4);
        if (count < 1) count = 1;
        if (count > 256) count = 256;
        addWriteRequest(ATIME, (byte) (256 - count));
    }

    public double getIntegrationTime() {
        return (double)(256 - aTimeValue) * 2.4;
    }

    public void setGain(int gain) {
        byte amplifierGain;
        if (gain < 4) {
            amplifierGain = CONTROL_AGAIN1;
        } else if (gain < 16) {
            amplifierGain = CONTROL_AGAIN4;
        } else if (gain < 60) {
            amplifierGain = CONTROL_AGAIN16;
        } else {
            amplifierGain = CONTROL_AGAIN60;
        }
        addWriteRequest(CONTROL, amplifierGain);
    }

    public int getGain() {
        if(controlValue == CONTROL_AGAIN1) return 1;
        if (controlValue == CONTROL_AGAIN4) return 4;
        if (controlValue == CONTROL_AGAIN16) return 16;
        return 60;
    }

    public void setLed(boolean state) {
        if (!state) {
            addWriteRequest(ENABLE, (byte) (ENABLE_AEIN | ENABLE_AEN | ENABLE_PON));
        } else {
            addWriteRequest(ENABLE, (byte) (ENABLE_AEN | ENABLE_PON));
        }
    }
}
