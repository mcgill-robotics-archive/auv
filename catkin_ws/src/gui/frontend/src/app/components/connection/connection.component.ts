import { Component, ElementRef, OnInit, ViewChild } from '@angular/core';
import {Subscription} from 'rxjs/Subscription';
import * as ROSLIB from 'roslib';

import { RosService } from 'app/services/ros.service';

@Component({
  selector: 'app-connection',
  templateUrl: './connection.component.html'
})
export class ConnectionComponent implements OnInit {

  // ROS
  url: string;
  connection: Subscription;

  constructor(private rosService: RosService) {
    this.connection = this.rosService.connection$.subscribe(data => {
      if (data) {
        this.notify(data);
      }
    });
  }

  ngOnInit() {
  }

  onConnectSubmit() {
    console.log(this.url);
  }

  notify(isConnected) {

  }

  ngOnDestroy() {
    this.connection.unsubscribe();
  }
}
