import { Component, ElementRef, OnInit, ViewChild } from '@angular/core';

@Component({
  selector: 'app-depth',
  templateUrl: './depth.component.html'
})
export class DepthComponent implements OnInit {

  @ViewChild('depthLevel') depthLevelRef: ElementRef;
  @ViewChild('setpointLevel') setpointLevelRef: ElementRef;

  depth: number = 4.4;
  setpoint: number = 5.2;

  readonly minDisplayDepth: number = 4.0;
  readonly maxDisplayDepth: number = 10.0;

  tick1: number = 0.0;
  tick2: number = 2.0;
  tick3: number = 4.0;
  tick4: number = 6.0;
  tick5: number = 8.0;

  constructor() { }

  ngOnInit() {
    this.update();
  }

  rescale() {
    // TODO Rescale the system based on the current depth and the setpoint
  }

  update() {
    var depthLevel = ((this.depth/this.tick5)*35) - 0.5;
    var setpointLevel = ((this.setpoint/this.tick5)*35) - 0.5;

    this.depthLevelRef.nativeElement.style.top = depthLevel + 'vh';
    this.setpointLevelRef.nativeElement.style.top = setpointLevel + 'vh';
  }
}
