package com.nordicsemi.nrfUARTv2;

import android.util.Log;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;

public class EpdDownloader {
    private static String TAG = "EpdDownloader";
    //private final static int STATE_ = 1;
    private  final  static int WIDTH = 104;
    private  final static int HEIGHT = 212;

    // buf of data to send
    private  byte[] black_buf  = new byte[WIDTH / 8 * HEIGHT];
    private  byte[] red_buf  = new byte[WIDTH / 8 * HEIGHT];

    private  int frame_index_black = 0;
    private  int frame_index_red = 0;
    public  final static int FRAME_DATA_SIZE = 13;
    // todo: check if the last frame is complete
    private  final static int TOTAL_FRAME_COUNT = WIDTH / 8 * HEIGHT / FRAME_DATA_SIZE;


    private  String errString;

    public final static byte EPD_CMD_PING = 0x10;
    public final static byte EPD_CMD_INIT = 0x11;
    public final static byte EPD_CMD_DEINIT = 0x12;

    public final static byte EPD_CMD_PREPARE_BLK_RAM = 0x13;
    public final static byte EPD_CMD_WRITE_BLK_RAM = 0x14;
    public final static byte EPD_CMD_GET_BLK_RAM_CRC = 0x15;
    public final static byte EPD_CMD_PREPARE_RED_RAM = 0x18;
    public final static byte EPD_CMD_WRITE_RED_RAM = 0x19;
    public final static byte EPD_CMD_GET_RED_RAM_CRC = 0x20;

    public final static byte EPD_CMD_UPDATE_DISPLAY = 0x21;  // ask epd to show ram data

    public EpdDownloader() {
        for (int i = 0; i < (WIDTH / 8 * HEIGHT) ; i++) {
            // fill with 0xff, (white color)
            black_buf[i] = (byte) 0xff;
            red_buf[i] = (byte) 0xff;
        }
    }

    public String getErrString() {
        return  errString;
    }

    public boolean loadFile(String filePath, boolean isBlack) {
        Log.i(TAG, "loading file: " + filePath);
        File file = new File(filePath);
        try {
            InputStream inputStream = new FileInputStream(file);

            if (isBlack) {
                inputStream.read(black_buf, 0, WIDTH / 8 * HEIGHT);
            } else {
                inputStream.read(red_buf, 0, WIDTH / 8 * HEIGHT);
            }

//            byte[] logBuf = new byte[10];
//            for (int i = 0; i < 10; i++) {
//                logBuf[i] = buff[i];
//            }
//            Log.i(TAG, "loaded, first ten bytes: " + bytesToHexString(logBuf));
            return true;

        } catch (FileNotFoundException e) {
            e.printStackTrace();
            errString = "file not found";
            return false;
        } catch (IOException e) {
            e.printStackTrace();
            errString = "ioException";
            return false;
        }
    }

    public void start() {
        // reset
        frame_index_black = 0;
        frame_index_red = 0;
    }

    public boolean isDataFinish(boolean isBlack) {
        if (isBlack) {
            return frame_index_black >= TOTAL_FRAME_COUNT;
        } else {
            return frame_index_red >= TOTAL_FRAME_COUNT;
        }

    }

    public  int getProgress(boolean isBlack) {
        int frm_idx = 0;
        if (isBlack) {
            frm_idx =  frame_index_black ;
        } else {
            frm_idx =  frame_index_red ;
        }

        if (frm_idx >= TOTAL_FRAME_COUNT) {
            return 100;
        } else {
            return (frm_idx * 100 / TOTAL_FRAME_COUNT);
        }
    }

    public byte[] getNextFrame(boolean isBlack) {
        byte[] ret = new byte[FRAME_DATA_SIZE];

        if (isBlack) {
            for (int i = 0; i < FRAME_DATA_SIZE; i++) {
                ret[i] = black_buf[FRAME_DATA_SIZE * frame_index_black + i];
            }
            frame_index_black++;
        } else {
            for (int i = 0; i < FRAME_DATA_SIZE; i++) {
                byte inverted = (byte) (0xff - red_buf[FRAME_DATA_SIZE * frame_index_red + i]);
                ret[i] = inverted;
            }
            frame_index_red ++;
        }

        return ret;
    }

    private static String bytesToHexString(byte[] src) {
        StringBuilder stringBuilder = new StringBuilder("");
        if (src == null || src.length <= 0) {
            return null;
        }
        for (int i = 0; i < src.length; i++) {
            int v = src[i] & 0xFF;
            String hv = Integer.toHexString(v);
            if (hv.length() < 2) {
                stringBuilder.append(0);
            }
            stringBuilder.append(hv);
            stringBuilder.append(' ');
        }
        return stringBuilder.toString();
    }

}
