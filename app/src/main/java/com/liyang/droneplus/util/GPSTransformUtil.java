package com.liyang.droneplus.util;

import java.math.BigDecimal;

public class GPSTransformUtil {

    public GPSTransformUtil() {
    }

    private double pi_GD = 3.14159265358979324;
    private double a = 6378245.0;
    private double ee = 0.00669342162296594323;

    public Gps gps84_To_Gcj02(double lat, double lon) {

        double dLat = transformLat(lon - 105.0, lat - 35.0);
        double dLon = transformLon(lon - 105.0, lat - 35.0);
        double radLat = lat / 180.0 * pi_GD;
        double magic = Math.sin(radLat);
        magic = 1 - ee * magic * magic;
        double sqrtMagic = Math.sqrt(magic);
        dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * pi_GD);
        dLon = (dLon * 180.0) / (a / sqrtMagic * Math.cos(radLat) * pi_GD);
        double mgLat = lat + dLat;
        double mgLon = lon + dLon;
        return new Gps(mgLat, mgLon);
    }

    public Gps gcj_To_Gps84(double lat, double lon) {
        Gps gps = transform(lat, lon);
        double lontitude = lon * 2 - gps.getWgLon();
        double latitude = lat * 2 - gps.getWgLat();
        return new Gps(latitude, lontitude);
    }

    public Gps transform(double lat, double lon) {

        double dLat = transformLat(lon - 105.0, lat - 35.0);
        double dLon = transformLon(lon - 105.0, lat - 35.0);
        double radLat = lat / 180.0 * pi_GD;
        double magic = Math.sin(radLat);
        magic = 1 - ee * magic * magic;
        double sqrtMagic = Math.sqrt(magic);
        dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * pi_GD);
        dLon = (dLon * 180.0) / (a / sqrtMagic * Math.cos(radLat) * pi_GD);
        double mgLat = lat + dLat;
        double mgLon = lon + dLon;
        return new Gps(mgLat, mgLon);
    }

    public double transformLat(double x, double y) {
        double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y
                + 0.2 * Math.sqrt(Math.abs(x));
        ret += (20.0 * Math.sin(6.0 * x * pi_GD) + 20.0 * Math.sin(2.0 * x * pi_GD)) * 2.0 / 3.0;
        ret += (20.0 * Math.sin(y * pi_GD) + 40.0 * Math.sin(y / 3.0 * pi_GD)) * 2.0 / 3.0;
        ret += (160.0 * Math.sin(y / 12.0 * pi_GD) + 320 * Math.sin(y * pi_GD / 30.0)) * 2.0 / 3.0;
        return ret;
    }

    public double transformLon(double x, double y) {
        double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1
                * Math.sqrt(Math.abs(x));
        ret += (20.0 * Math.sin(6.0 * x * pi_GD) + 20.0 * Math.sin(2.0 * x * pi_GD)) * 2.0 / 3.0;
        ret += (20.0 * Math.sin(x * pi_GD) + 40.0 * Math.sin(x / 3.0 * pi_GD)) * 2.0 / 3.0;
        ret += (150.0 * Math.sin(x / 12.0 * pi_GD) + 300.0 * Math.sin(x / 30.0
                * pi_GD)) * 2.0 / 3.0;
        return ret;
    }

    public class Gps {

        private double wgLat;
        private double wgLon;

        public Gps(double wgLat, double wgLon) {
            setWgLat(wgLat);
            setWgLon(wgLon);
        }

        public double getWgLat() {
            BigDecimal b = new BigDecimal(wgLat);
            wgLat = b.setScale(8, BigDecimal.ROUND_DOWN).doubleValue();
            return wgLat;
        }

        public void setWgLat(double wgLat) {
            this.wgLat = wgLat;
        }

        public double getWgLon() {
            BigDecimal b = new BigDecimal(wgLon);
            wgLon = b.setScale(8, BigDecimal.ROUND_DOWN).doubleValue();
            return wgLon;
        }

        public void setWgLon(double wgLon) {
            this.wgLon = wgLon;
        }

        @Override
        public String toString() {
            return wgLat + "," + wgLon;
        }
    }
}
