import { BrowserModule } from '@angular/platform-browser';
import { NgModule } from '@angular/core';
import { RouterModule, Routes } from '@angular/router';
import { FlashMessagesModule } from 'angular2-flash-messages';

import { environment } from '../environments/environment';

import { RosService } from 'app/services/ros.service';

import { AppComponent } from './app.component';
import { NavbarComponent } from './components/navbar/navbar.component';
import { HomeComponent } from './components/home/home.component';
import { TestComponent } from './components/test/test.component';
import { CompassComponent } from './components/compass/compass.component';
import { BatteryComponent } from './components/battery/battery.component';
import { DepthComponent } from './components/depth/depth.component';

const routes: Routes = [
  {path: '', component: HomeComponent},
  {path: 'test', component: TestComponent},
  {path: '**', redirectTo:''}
]

@NgModule({
  declarations: [
    AppComponent,
    NavbarComponent,
    HomeComponent,
    TestComponent,
    CompassComponent,
    BatteryComponent,
    DepthComponent
  ],
  imports: [
    BrowserModule,
    RouterModule.forRoot(routes),
    FlashMessagesModule.forRoot(),
  ],
  providers: [
    RosService
  ],
  bootstrap: [
    AppComponent
  ]
})
export class AppModule { }
