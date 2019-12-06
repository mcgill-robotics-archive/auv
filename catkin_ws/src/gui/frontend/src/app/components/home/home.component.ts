import { Component, OnInit, Output } from '@angular/core';
import { EventEmitter } from 'events';

@Component({
  selector: 'app-home',
  templateUrl: './home.component.html'
})
export class HomeComponent implements OnInit {

  data;

  constructor() { 
    this.data = {
      voltage: 0,
      depth: 0,
      status: false,
      orientation: 0
    };
  }
  //@Output() dataMade = new EventEmitter();

  ngOnInit(){
    this.data = this.onClick();
  }

  ngDestroy(){

  }

  onClick(){
    const data = {
      voltage: 10,
      depth: 25,
      On: true,
      orientation: 0
    }
    
    return data;
  }

}
