import {NgModule} from '@angular/core';
import {BrowserModule} from '@angular/platform-browser';

import {App} from './app';
import {CameraTile} from './camera_tile';
import {DistortionMap} from './distortion_map';
import {IntrinsicsCard} from './intrinsics_card';

@NgModule({
  declarations: [App, CameraTile, DistortionMap, IntrinsicsCard],
  imports: [BrowserModule],
  bootstrap: [App],
})
export class AppModule {}
