import { Component, OnInit } from '@angular/core';
import { Router } from '@angular/router';

import { FlashMessagesService } from 'angular2-flash-messages';

@Component({
  selector: 'app-navbar',
  host: {'(document:click)': 'onOutsideClicked($event)'},
  templateUrl: './navbar.component.html'
})

export class NavbarComponent implements OnInit {

  constructor(private router: Router) { }

  ngOnInit() {
  }

  onOutsideClicked(event) {
    var dropdown = document.getElementById('navbarNav');
    if (dropdown.classList.contains('show')) {
      dropdown.classList.remove('show');
    }
  }
}
