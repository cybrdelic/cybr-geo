    const requestI=clip(grossRequest/(gr.gear_ratio*gr.torque_constant_Nm_A*etaRequest),-this.gripperCurrentCeiling*o.gripScale,this.gripperCurrentCeiling*o.gripScale);
    const emf=gr.back_emf_V_s_rad*rotorSpeed;
    this.gripperVoltage=clip(gr.resistance_ohm*requestI+emf,-gr.bus_voltage_V,gr.bus_voltage_V);
    if(!this.gripperPower){this.gripperCurrent=0;this.gripperVoltage=0;}
    else{const eq=(this.gripperVoltage-emf)/gr.resistance_ohm;this.gripperCurrent=eq+(this.gripperCurrent-eq)*Math.exp(-gr.resistance_ohm*dt/gr.inductance_H);}
    this.gripperCurrent=clip(this.gripperCurrent,-this.gripperCurrentCeiling*o.gripScale,this.gripperCurrentCeiling*o.gripScale);
    const etaActual=Math.exp(Math.log(gr.gear_efficiency)*gripTransition*Math.sign(this.gripperCurrent));
    this.gripperTorque=gr.gear_ratio*gr.torque_constant_Nm_A*this.gripperCurrent*etaActual;ctrl[6]=this.gripperTorque;
    const gripHeat=this.gripperCurrent*this.gripperCurrent*gr.resistance_ohm;
    this.gripperTemperature+=(gripHeat-(this.gripperTemperature-25)/gr.thermal_resistance_K_W)*dt/gr.thermal_capacity_J_K;
    this.gripperI2Integral+=this.gripperCurrent*this.gripperCurrent*dt;