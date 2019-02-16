import { Component, ElementRef, ViewChild, OnInit } from '@angular/core';
import {Subscription} from 'rxjs/Subscription';
import * as ROSLIB from 'roslib';

import { RosService } from 'app/services/ros.service';

@Component({
  selector: 'app-compass',
  templateUrl: './compass.component.html'
})
export class CompassComponent implements OnInit {

  @ViewChild('setpointArrow') setpointArrowRef: ElementRef;
  @ViewChild('headingArrow') headingArrowRef: ElementRef;
  @ViewChild('pingerBlip') pingerBlipRef: ElementRef;

  // Data
  heading: number = 0;
  setpoint: number = 1.57;
  pinger: number = 3.14;

  // Data | UI
  headingDeg: number = 180;
  setpointDeg: number = 45;
  pingerDeg: number = 315;

  // ROS
  connection: Subscription;
  headingSub: any;
  setpointSub: any;
  pingerSub: any;

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
    this.headingSub = new ROSLIB.Topic({
      ros : this.rosService.getRos(),
      name : '/state_estimation/pose',
      messageType : 'geometry_msgs/PoseStamped'
    });
    this.headingSub.subscribe(function(message) {
      this.heading = message.pose.orientation.z;
      this.update();
    }.bind(this));

    this.pingerSub = new ROSLIB.Topic({
      ros : this.rosService.getRos(),
      name : '/hydrophones/heading',
      messageType : 'std_msgs/Float64'
    });
    this.pingerSub.subscribe(function(message) {
      this.pinger = message.data;
      this.update();
    }.bind(this));
  };

  detach() {
    // TODO unsubscribe from ROS topic
  }

  rad2Deg(rad) {
    return Math.round((rad/6.28318530718)*360)
  }

  update() {
    // TODO mmight be better to move this elsewhere when the rest of
    // the system is implemented
    this.headingDeg = this.rad2Deg(this.heading);
    this.setpointDeg = this.rad2Deg(this.setpoint);
    this.pingerDeg = this.rad2Deg(this.pinger);
    // END TODO

    this.headingArrowRef.nativeElement.style.transform =
        'rotate(' + this.headingDeg + 'deg)';
    this.setpointArrowRef.nativeElement.style.transform =
        'rotate(' + this.setpointDeg + 'deg)';
    if (this.pinger > 0) {
      this.pingerBlipRef.nativeElement.style.visibility = 'visible';
      this.pingerBlipRef.nativeElement.style.transform =
          'rotate(' + this.pingerDeg + 'deg)';
    } else {
      this.pingerBlipRef.nativeElement.style.visibility = 'hidden';
    }
  }

  ngOnDestroy() {
    this.connection.unsubscribe();
  }
}
