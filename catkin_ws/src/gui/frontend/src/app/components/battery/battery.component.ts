import { Component, OnInit } from '@angular/core';

@Component({
  selector: 'app-battery',
  templateUrl: './battery.component.html'
})
export class BatteryComponent implements OnInit {

  voltage: number = 14.2;

  constructor() { }

  ngOnInit() {
  }

}
