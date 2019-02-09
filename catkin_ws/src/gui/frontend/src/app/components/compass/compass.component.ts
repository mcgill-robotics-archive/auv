import { Component, ElementRef, ViewChild, AfterViewInit } from '@angular/core';

@Component({
  selector: 'app-compass',
  templateUrl: './compass.component.html'
})
export class CompassComponent implements AfterViewInit {

  @ViewChild('setpointArrow') setpointArrowRef: ElementRef;
  @ViewChild('headingArrow') headingArrowRef: ElementRef;
  @ViewChild('pingerBlip') pingerBlipRef: ElementRef;

  heading: number = 180;
  setpoint: number = 45;
  pinger: number = 315;

  constructor() { }

  ngAfterViewInit() {
    this.update();
  }

  update() {
    this.setpointArrowRef.nativeElement.style.transform =
        'rotate(' + this.setpoint + 'deg)';
    this.headingArrowRef.nativeElement.style.transform =
        'rotate(' + this.heading + 'deg)';
    if (this.pinger > 0) {
      this.pingerBlipRef.nativeElement.style.visibility = 'visible';
      this.pingerBlipRef.nativeElement.style.transform =
          'rotate(' + this.pinger + 'deg)';
    } else {
      this.pingerBlipRef.nativeElement.style.visibility = 'hidden';
    }
  }
}
