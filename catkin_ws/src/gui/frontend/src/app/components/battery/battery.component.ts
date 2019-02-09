import { Component, ElementRef, OnInit, ViewChild } from '@angular/core';
import {Subscription} from 'rxjs/Subscription';
import * as ROSLIB from 'roslib';

import { RosService } from 'app/services/ros.service';

@Component({
  selector: 'app-battery',
  templateUrl: './battery.component.html'
})
export class BatteryComponent implements OnInit {

  @ViewChild('batLevel') voltageGaugeRef: ElementRef;

  // Data
  voltage: number = 0;

  // Limits
  readonly minVoltage: number = 13;
  readonly maxVoltage: number = 17;

  // UI
  readonly maxGaugeSize: number = 15.5;
  voltageGauge: number = 0;

  // ROS
  connection: Subscription;
  voltageSub: any;

  constructor(private rosService: RosService) {
    this.connection = this.rosService.connection$.subscribe(data => {
      if (data) {
        this.listen();
      }
    });
  }

  ngOnInit() {
    this.update();
  }

  listen() {
    this.voltageSub = new ROSLIB.Topic({
      ros : this.rosService.getRos(),
      name : '/dcdc_nuc/input_voltage',
      messageType : 'std_msgs/Float64'
    });
    this.voltageSub.subscribe(function(message) {
      this.voltage = message.data;
      this.update();
    }.bind(this));
  };

  detach() {
    // TODO unsubscribe from ROS topic
  }

  update() {
    this.voltageGauge = ((this.voltage - this.minVoltage) /
        (this.maxVoltage - this.minVoltage)) * this.maxGaugeSize;

    this.voltageGaugeRef.nativeElement.style.width = this.voltageGauge + 'vh';
  }

  ngOnDestroy() {
    this.connection.unsubscribe();
  }
}
