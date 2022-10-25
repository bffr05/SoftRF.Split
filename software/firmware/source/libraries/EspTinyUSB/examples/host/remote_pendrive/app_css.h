const char* app_css = 
".main-content {"
"    width: 1440px;"
"    margin: auto;"
"    font-size: 14px;"
"  }"
"  "
"  .connect-container {"
"    margin: 20px 0;"
"  }"
"  "
"  .container {"
"    display: flex;"
"  }"
"  "
"  .sender, .receiver {"
"    flex-grow: 0.5;"
"    flex-shrink: 1;"
"  }"
"  "
"  .sender {"
"    margin-right: 8px;"
"  }"
"  "
"  .receiver {"
"    margin-right: 8px;"
"  }"
"  "
"  .lines-header {"
"    height: 30px;"
"    width: 100%;"
"    box-sizing: border-box;"
"    background-color: #444;"
"    line-height: 30px;"
"    color: white;"
"    padding-left: 10px;"
"  }"
"  "
"  .lines-body {"
"    width: 100%;"
"    background-color: #222;"
"    min-height: 300px;"
"    padding: 10px 0 20px 0;"
"  }"
"  "
"  .line, .command-line {"
"    box-sizing: border-box;"
"    width: 100%;"
"    color: #f1f1f1;"
"    background-color: #222;"
"    outline: none;"
"    border: none;"
"    padding: 5px 10px;"
"    font-size: 14px;"
"  }"
"  "
"  .line:hover {"
"     background-color: #444;"
"  }"
"  "
"  .button::before {"
"      -webkit-border-radius: 3px;"
"      -moz-border-radius: 3px;"
"      -webkit-box-shadow: #959595 0 2px 5px;"
"      -moz-box-shadow: #959595 0 2px 5px;"
"      border-radius: 3px;"
"      box-shadow: #959595 0 2px 5px;"
"      content: '';"
"      display: block;"
"      left: 0;"
"      padding: 2px 0 0;"
"      position: absolute;"
"      top: 0;"
"  }"
"  "
"  .button:active::before { padding: 1px 0 0; }"
"  "
"  .button.black {"
"      background: #656565;"
"      background: -webkit-gradient(linear, 0 0, 0 bottom, from(#656565), to(#444));"
"      background: -moz-linear-gradient(#656565, #444);"
"      background: linear-gradient(#656565, #444);"
"      border: solid 1px #535353;"
"      border-bottom: solid 3px #414141;"
"      box-shadow: inset 0 0 0 1px #939393;"
"      color: #fff;"
"      text-shadow: 0 1px 0 #2f2f2f;"
"      padding: 8px 16px;"
"      outline: none;"
"  }"
"  "
"  .button.black:hover {"
"      background: #4c4c4c;"
"      background: -webkit-gradient(linear, 0 0, 0 bottom, from(#4c4c4c), to(#565656));"
"      background: -moz-linear-gradient(#4c4c4c, #565656);"
"      background: linear-gradient(#4c4c4c, #565656);"
"      border: solid 1px #464646;"
"      border-bottom: solid 3px #414141;"
"      box-shadow: inset 0 0 0 1px #818181;"
"  }"
"  "
"  .button.black:active {"
"      background: #474747;"
"      background: -webkit-gradient(linear, 0 0, 0 bottom, from(#474747), to(#444));"
"      background: -moz-linear-gradient(#474747, #444);"
"      background: linear-gradient(#474747, #444);"
"      border: solid 1px #2f2f2f;"
"      box-shadow: inset 0 10px 15px 0 #3e3e3e;"
"  }"
"  "
"  "
"  .filestree {"
"    border: 1px solid black;"
"    width: 100%;"
"  }"
"  "
"  .filestree1 {"
"    border: 1px solid black;"
"    width: 100%;"
"    text-align: end;"
"  }"
"  "
"  th {"
"    border: 1px solid black;"
"    width: 100%;"
"  }"
"  "
"body {"
"  background-color: #125597;"
"  /* background-image: url('https://cdn.shopify.com/s/files/1/0770/0935/files/nasa-53884_1512x.jpg'); */"
"  text-decoration-color: white;"
"  background-size: cover;"
"  font-family: 'Helvetica Neue', Arial, sans-serif;"
"}"
""
"* {"
"  box-sizing: border-box;"
"}"
""
".card {"
"  background: linear-gradient(-45deg, #5555FF, #9787FF);"
"  box-shadow: 0 0 6px rgba(0, 0, 0, 0.1); "
"  position: absolute;"
"  top: 50%;"
"  left: 50%;"
"  transform: translate(-50%, -50%);"
"  width: 300px;"
"  height: 375px;"
"  border-radius: 10px;"
"  overflow: hidden;"
"}"
""
"/* hide limit values on X axis */"
".card #canvas {"
"  margin-left: -30px;"
"  margin-right: -30px;"
"  width: 360px!important;"
"}"
""
".card .about {"
"  height: 185px;"
"  padding: 20px;"
"  box-sizing: border-box;"
"}"
""
".card .about h3,"
".card .about .lead {"
"  margin-top: 0;"
"  margin-bottom: 0;"
"  font-weight: 400;"
"}"
""
".card .about h3 {"
"  font-size: 24px;"
"  color: #fff;"
"}"
""
".card .about .lead {"
"  color: #eee;"
"}"
""
".card .info {"
"  float: left;"
"  padding: 10px 30px 10px 0;"
"}"
""
".card .info p {"
"  font-size: 11px;"
"  color: #aaa;"
"  font-weight: 300;"
"}"
""
".legends {"
"  padding-top: 20px;"
"  overflow: hidden;"
"}"
""
".legend {"
"  display: block;"
"  width: 8px;"
"  height: 8px;"
"  margin-top: 15px;"
"  margin-bottom: 15px;"
"  border-radius: 50%;"
"}"
""
".legend--this {"
"  background-color: #5555FF;"
"}"
""
".legend--prev {"
"  background-color: #FF55B8;"
"}"
""
".axis {"
"  position: absolute;"
"  color: #fff;"
"  z-index: 1;"
"  text-transform: uppercase;"
"  display: flex;"
"  width: 100%;"
"  bottom: 0;"
"}"
""
".axis .tick {"
"  flex: 1;"
"  position: relative;"
"  font-size: 11px;"
"  text-align: center;"
"  padding-top: 10px;"
"  padding-bottom: 10px;"
"  line-height: 20px;"
"}"
""
".axis .tick::after {"
"  content: '';"
"  position: absolute;"
"  display: block;"
"  right: 0;"
"  bottom: 0;"
"  width: 1px;"
"  height: 200px;"
"  background: rgba(255, 255, 255, 0.2);"
"}"
""
".axis .tick .value {"
"  transform: translateY(-240px);"
"  opacity: 0;"
"  transition: all 0.3s;"
"  position: absolute;"
"  top: 20px;"
"  left: 0;"
"  color: #fff;"
"  border-radius: 2px;"
"  width: 100%;"
"  line-height: 20px;"
"}"
""
".axis .tick:hover .value.value--this {"
"  transform: translateY(-160px);"
"  display: block;"
"  opacity: 0.4;"
"}"
""
".value.value--this {"
"  color: #fff;"
"  font-weight: bold;"
"}"
""
".day-number {"
"  display: block;"
"}"
""
".day-name {"
"  display: block;"
"  opacity: 0.4;"
"}"
""
"/* Animated background, credits: Manuel Pinto, https://codepen.io/P1N2O/pen/pyBNzX */"
".card {"
"  background: linear-gradient(-45deg, #5555FF, #9787FF, #FF55B8, #FF8787);"
"  background-size: 400% 400%;"
"  animation: bg 20s infinite;"
"}"
""
"@keyframes bg"
"{"
"  0% {"
"		background-position: 0% 50%"
"	}"
"	50% {"
"		background-position: 100% 50%"
"	}"
"	100% {"
"		background-position: 0% 50%"
"	}"
"}"
""
""
""
"/* The Modal (background) */"
".modal {"
"  display: none; /* Hidden by default */"
"  position: fixed; /* Stay in place */"
"  z-index: 1; /* Sit on top */"
"  left: 0;"
"  top: 0;"
"  width: 100%; /* Full width */"
"  height: 100%; /* Full height */"
"  overflow: auto; /* Enable scroll if needed */"
"  background-color: rgb(0,0,0); /* Fallback color */"
"  background-color: rgba(0,0,0,0.4); /* Black w/ opacity */"
"}"
""
"/* Modal Content/Box */"
".modal-content {"
"  background-color: #fefefe;"
"  margin: 10% 10% auto; /* 15% from the top and centered */"
"  padding: 20px;"
"  border: 1px solid #888;"
"  width: 80%; /* Could be more or less, depending on screen size */"
"}"
""
"/* The Close Button */"
".close {"
"  color: #aaa;"
"  float: right;"
"  font-size: 28px;"
"  font-weight: bold;"
"}"
""
".close:hover,"
".close:focus {"
"  color: black;"
"  text-decoration: none;"
"  cursor: pointer;"
"}"
""
"footer {"
"    position: absolute;"
"    bottom: 0px;"
"}";