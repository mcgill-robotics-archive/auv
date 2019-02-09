import { Component, ElementRef, OnInit, ViewChild } from '@angular/core';
import {Subscription} from 'rxjs/Subscription';
import * as ROSLIB from 'roslib';

import { RosService } from 'app/services/ros.service';

@Component({
  selector: 'app-depth',
  templateUrl: './depth.component.html'
})
export class DepthComponent implements OnInit {

  @ViewChild('depthLevel') depthLevelRef: ElementRef;
  @ViewChild('setpointLevel') setpointLevelRef: ElementRef;

  // Data
  depth: number = 4.4;
  setpoint: number = 5.2;

  // UI
  readonly minDisplayDepth: number = 4.0;
  readonly maxDisplayDepth: number = 10.0;
  depthLevel: number = 0;
  setpointLevel: number = 0;
  tick1: number = 0.0;
  tick2: number = 2.0;
  tick3: number = 4.0;
  tick4: number = 6.0;
  tick5: number = 8.0;

  // ROS
  connection: Subscription;
  depthSub: any;
  setpointSub: any;

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
    this.depthSub = new ROSLIB.Topic({
      ros : this.rosService.getRos(),
      name : '/state_estimation/depth',
      messageType : 'std_msgs/Float64'
    });
    this.depthSub.subscribe(function(message) {
      this.depth = message.data;
      this.update();
    }.bind(this));
  };

  detach() {
    // TODO unsubscribe from ROS topic
  }

  rescale() {
    // TODO Rescale the system based on the current depth and the setpoint
  }

  update() {
    this.depthLevel = ((this.depth/this.tick5)*35) - 0.5;
    this.setpointLevel = ((this.setpoint/this.tick5)*35) - 0.5;

    this.depthLevelRef.nativeElement.style.top = this.depthLevel + 'vh';
    this.setpointLevelRef.nativeElement.style.top = this.setpointLevel + 'vh';
  }

  ngOnDestroy() {
    this.connection.unsubscribe();
  }
}
