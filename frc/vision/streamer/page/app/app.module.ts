import {NgModule} from '@angular/core';
import {FormsModule} from '@angular/forms';
import {BrowserModule} from '@angular/platform-browser';

import {App} from './app';
import {CalibratePanel} from './calibrate';
import {CameraTile} from './camera_tile';
import {DistortionMap} from './distortion_map';
import {IntrinsicsCard} from './intrinsics_card';
import {SamplesView} from './samples_view';
import {ValidationGridView} from './validation_grid';

@NgModule({
  declarations: [
    App,
    CalibratePanel,
    CameraTile,
    DistortionMap,
    IntrinsicsCard,
    SamplesView,
    ValidationGridView,
  ],
  imports: [BrowserModule, FormsModule],
  bootstrap: [App],
})
export class AppModule {}
