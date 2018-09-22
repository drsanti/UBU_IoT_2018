

//!!--------------------------------------------------------------

//!! Homepage, 2003 bytes (can be up to 2048 bytes)

//>> HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n 
//const char * indexHtml="HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n <html><link rel='icon'href='data:;base64,iVBORw0KGgo='><head><link rel='stylesheet'href='http://ecc-lab.com/shared/bootstrap.css'><script src='http://ecc-lab.com/shared/jquery.js'></script><script src='http://ecc-lab.com/shared/bootstrap.js'></script><link rel='stylesheet'type='text/css' href='./index.css'><script src='index.js'></script></head><body><center><div class='container'><div class='row box'><div class='col col-xs-12'><div class='g-name'>Toggle LED</div><button class='btn btn-warning'id='led0-inv'>LED0 Inv</button><button class='btn btn-warning'id='led1-inv'>LED1 Inv</button><button class='btn btn-warning'id='led2-inv'>LED2 Inv</button><button class='btn btn-warning'id='led3-inv'>LED3 Inv</button></div></div><div class='row box'><div class='col col-xs-12'><div class='g-name'>Torn ON LED</div><button class='btn btn-danger'id='led0-on'>LED0 On</button><button class='btn btn-danger'id='led1-on'>LED1 On</button><button class='btn btn-danger'id='led2-on'>LED2 On</button><button class='btn btn-danger'id='led3-on'>LED3 On</button></div></div><div class='row box'><div class='col col-xs-12'><div class='g-name'>Torn OFF LED</div><button class='btn btn-primary'id='led0-off'>LED0 Off</button><button class='btn btn-primary'id='led1-off'>LED1 Off</button><button class='btn btn-primary'id='led2-off'>LED2 Off</button><button class='btn btn-primary'id='led3-off'>LED3 Off</button></div></div><div class='row box'><div class='col col-xs-12'><div class='g-name'>Read ADC</div><button class='btn btn-success'id='adc0'>ADC0</button><button class='btn btn-success'id='adc1'>ADC1</button><button class='btn btn-success'id='adc2'>ADC2</button><button class='btn btn-success'id='adc3'>ADC3</button></div></div><div class='row box'><div class='col col-xs-12'><div class='result'id='adc'></div></div></div></div></center></body></html>";
const char * indexHtml="HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<html><head><title>DEV10</title><link rel='icon'href='data:;base64,iVBORw0KGgo='><link rel='icon'href='data:;base64,iVBORw0KGgo='><link rel='stylesheet'href='http://ecc-lab.com/shared/bootstrap.css'><script src='http://ecc-lab.com/shared/jquery.js'></script><script src='http://ecc-lab.com/shared/bootstrap.js'></script><link rel='stylesheet'type='text/css' href='./index.css'><script src='index.js'></script></head><body><center><div class='container'><div class='row box'><div class='col col-xs-12'><div class='g-name'>Toggle LED</div><button class='btn btn-warning'id='led0-inv'>LED0 Inv</button><button class='btn btn-warning'id='led1-inv'>LED1 Inv</button><button class='btn btn-warning'id='led2-inv'>LED2 Inv</button><button class='btn btn-warning'id='led3-inv'>LED3 Inv</button></div></div><div class='row box'><div class='col col-xs-12'><div class='g-name'>Torn ON LED</div><button class='btn btn-danger'id='led0-on'>LED0 On</button><button class='btn btn-danger'id='led1-on'>LED1 On</button><button class='btn btn-danger'id='led2-on'>LED2 On</button><button class='btn btn-danger'id='led3-on'>LED3 On</button></div></div><div class='row box'><div class='col col-xs-12'><div class='g-name'>Torn OFF LED</div><button class='btn btn-primary'id='led0-off'>LED0 Off</button><button class='btn btn-primary'id='led1-off'>LED1 Off</button><button class='btn btn-primary'id='led2-off'>LED2 Off</button><button class='btn btn-primary'id='led3-off'>LED3 Off</button></div></div><div class='row box'><div class='col col-xs-12'><div class='g-name'>Read ADC</div><button class='btn btn-success'id='adc0'>ADC0</button><button class='btn btn-success'id='adc1'>ADC1</button><button class='btn btn-success'id='adc2'>ADC2</button><button class='btn btn-success'id='adc3'>ADC3</button></div></div><div class='row box'><div class='col col-xs-12'><div class='result'id='adc'><h1 style='color:aquamarine;'>ECC-Lab</h1></div></div></div></div></center></body></html>";


//!! CSS
//>> HTTP/1.1 200 OK\r\nContent-Type: text/css\r\nConnection: close\r\n\r\n 
const char * indexCss="HTTP/1.1 200 OK\r\nContent-Type: text/css\r\n\r\nbody{margin:0;padding:0;background-color:#223344;}.g-name{color:grey;font-size:2.0em;margin-top:0.5em;margin-bottom:0.5em;}.box{border:2px solid gray;border-radius:10px;padding-bottom:3em;margin-top:3em;}.btn{width:7em;height:3.5em;font-weight:bold;margin-left:0.5em;margin-right:0.5em;font-size:1.5em;}.btn-danger,.btn-warning{color:black;}.result{font-weight:bold;margin-top:5em;}";



//!! JS
//>> HTTP/1.1 200 OK\r\nContent-Type: text/javascript\r\n\r\n 
const char * indexJs="HTTP/1.1 200 OK\r\nContent-Type: text/javascript\r\n\r\nwindow.onload=function(e){function getEle(e){return document.getElementById(e);}function load(req){var xml=new XMLHttpRequest();xml.timeout=2000;xml.onreadystatechange=function(){if (xml.readyState==XMLHttpRequest.DONE){if (xml.status==200){getEle('adc').innerHTML=xml.responseText;}else if (xml.status==400){console.log('There was an error 400');}else{console.log('something else other than 200 was returned');}}};xml.open('GET',req,true);xml.send();}function doIt(id,cmd){getEle(id).addEventListener('click',function(){load(cmd);});}for(var i=0;i<4;i++){doIt('led'+i+'-inv','led/'+i+'/2;');doIt('led'+i+'-on','led/'+i+'/1;');doIt('led'+i+'-off','led/'+i+'/0;');doIt('adc'+i,'adc/'+i+'/0;');}setInterval(function(){load('adc/1/0;');},3000);}";


//!! favicon.ico
const char *faviconIco = "HTTP/1.1 200 OK\r\nContent-Type: image/png\r\n\r\nECC-Lab";





