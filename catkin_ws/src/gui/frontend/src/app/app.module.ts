import { BrowserModule } from '@angular/platform-browser';
import { NgModule } from '@angular/core';
import { RouterModule, Routes } from '@angular/router';
import { FormsModule } from '@angular/forms';
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
import { ConnectionComponent } from './components/connection/connection.component';
import { CameraComponent } from './components/camera/camera.component';
import { MissionComponent } from './components/mission/mission.component';
import { VisionComponent } from './components/vision/vision.component';

const routes: Routes = [
  {path: '', component: HomeComponent},
  {path: 'mission', component: MissionComponent},
  {path: 'vision', component: VisionComponent},
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
    DepthComponent,
    ConnectionComponent,
    CameraComponent,
    MissionComponent,
    VisionComponent
  ],
  imports: [
    BrowserModule,
    RouterModule.forRoot(routes),
    FormsModule,
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
