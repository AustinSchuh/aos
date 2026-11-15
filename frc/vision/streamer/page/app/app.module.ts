import {NgModule} from '@angular/core';
import {BrowserModule} from '@angular/platform-browser';

import {App} from './app';
import {CameraTile} from './camera_tile';
import {IntrinsicsCard} from './intrinsics_card';

@NgModule({
  declarations: [App, CameraTile, IntrinsicsCard],
  imports: [BrowserModule],
  bootstrap: [App],
})
export class AppModule {}
