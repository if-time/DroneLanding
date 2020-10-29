package com.liyang.droneplus.util;

import com.liyang.droneplus.entity.PIDEntityInc;

public class PIDControlIncTest {

    private PIDEntityInc pidEntityInc;

    public void initPID(float p, float i, float d) {
        pidEntityInc = new PIDEntityInc();
        pidEntityInc.setSetValue(0);
        pidEntityInc.setActualValue(0);
        pidEntityInc.setErr(0);
        pidEntityInc.setLastErr(0);
        pidEntityInc.setBeforLastErr(0);
        pidEntityInc.setKp(p);
        pidEntityInc.setKi(i);
        pidEntityInc.setKd(d);
        pidEntityInc.setControlValue(0);
    }

    public float pidRealize(float distance) {
        pidEntityInc.setActualValue(distance);

        pidEntityInc.setErr(pidEntityInc.getSetValue()-pidEntityInc.getActualValue());
        // 控制量的增量
        float controlValue = pidEntityInc.getKp()*(pidEntityInc.getErr()-pidEntityInc.getLastErr())
                + pidEntityInc.getKi()*pidEntityInc.getErr()
                + pidEntityInc.getKd()*(pidEntityInc.getErr() - 2*pidEntityInc.getLastErr() + pidEntityInc.getBeforLastErr());
        // 控制量 = 上一次的控制量 + 控制量的增量
        pidEntityInc.setControlValue(pidEntityInc.getControlValue()+controlValue);
        pidEntityInc.setBeforLastErr(pidEntityInc.getLastErr());

        pidEntityInc.setLastErr(pidEntityInc.getErr());

        return pidEntityInc.getControlValue(); // 返回控制量
    }

}
