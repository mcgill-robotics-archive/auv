import { Component, ElementRef, ViewChild, OnInit } from '@angular/core';

@Component({
  selector: 'app-compass',
  templateUrl: './compass.component.html'
})
export class CompassComponent implements OnInit {

  @ViewChild('setpointArrow') setpointArrowRef: ElementRef;
  @ViewChild('headingArrow') headingArrowRef: ElementRef;
  @ViewChild('pingerBlip') pingerBlipRef: ElementRef;

  heading: number = 1.57;
  setpoint: number = 3.14;
  pinger: number = 5.62;

  heading_deg: number = 180;
  setpoint_deg: number = 45;
  pinger_deg: number = 315;

  constructor() { }

  ngOnInit() {
    this.update();
  }

  rad2Deg(rad) {
    return Math.round((rad/6.28318530718)*360)
  }

  update() {
    // TODO mmight be better to move this elsewhere when the rest of
    // the system is implemented
    this.heading_deg = this.rad2Deg(this.heading);
    this.setpoint_deg = this.rad2Deg(this.setpoint);
    this.pinger_deg = this.rad2Deg(this.pinger);
    // END TODO

    this.headingArrowRef.nativeElement.style.transform =
        'rotate(' + this.heading_deg + 'deg)';
    this.setpointArrowRef.nativeElement.style.transform =
        'rotate(' + this.setpoint_deg + 'deg)';
    if (this.pinger > 0) {
      this.pingerBlipRef.nativeElement.style.visibility = 'visible';
      this.pingerBlipRef.nativeElement.style.transform =
          'rotate(' + this.pinger_deg + 'deg)';
    } else {
      this.pingerBlipRef.nativeElement.style.visibility = 'hidden';
    }
  }
}
