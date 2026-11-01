    if(Math.abs(this.gripperCurrent)>.1875+1e-5)this.peakCredit=Math.max(0,this.peakCredit-dt);
    else if(Math.abs(this.gripperCurrent)<.8*.1875)this.peakCredit=Math.min(this.spec.gripper_driver.peak_budget_s,this.peakCredit+dt);
    this.gripperCurrentCeiling=this.peakCredit>0?this.spec.gripper_driver.peak_current_A:this.spec.gripper_driver.continuous_current_A;
