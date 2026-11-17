import {Connection} from '../../aos/network/www/proxy';

import {StarterHandler} from './starter_handler';

const conn = new Connection();

conn.connect();

new StarterHandler(conn);
