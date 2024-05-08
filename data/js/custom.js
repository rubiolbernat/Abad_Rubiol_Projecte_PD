// Locate the 3 checkbox
const nunchuk = document.querySelector('#nunchuk-button');
const tube = document.querySelector('#tube');
const bulb = document.querySelector('#bulb');
const socket = document.querySelector('#socket');

// Add event Listener to the 3 checkbox
nunchuk.addEventListener('change', toggleStatus);
tube.addEventListener('change', toggleStatus);
bulb.addEventListener('change', toggleStatus);
socket.addEventListener('change', toggleStatus);

async function sendRequestToServer(url) {
  try {
    let res = await fetch(url);
    return await res.json();
  } catch (error) {
    console.log(error);
  }
}

async function toggleStatus(e) {
  let sourceElementName = e.target.name;
  let url = '/toggle/' + sourceElementName + '?status=';
  if (e.target.checked) {
    url += 'true';
  } else {
    url += 'false';
  }
  console.log("Sending to " + url);

  let response = await sendRequestToServer(url);

  console.log(response);
}


const servo1Slider = document.querySelector("#servo1-slider");
const servo2Slider = document.querySelector("#servo2-slider");
const servo3Slider = document.querySelector("#servo3-slider");
const servo4Slider = document.querySelector("#servo4-slider");

// Add event on servo1 slider change
servo1Slider.addEventListener('input', updateServo1Value);

async function updateServo1Value(e) {
    let url = '/servo1/change' + '?value=' + servo1Slider.value;
    console.log("Sending to " + url);

    let response = await sendRequestToServer(url);

    console.log(response);

    //update servo1 value
    document.querySelector("#servo1-value").innerHTML = servo1Slider.value;
}

// Add event on servo2 slider change
servo2Slider.addEventListener('input', updateServo2Value);

async function updateServo2Value(e) {
    let url = '/servo2/change' + '?value=' + servo2Slider.value;
    console.log("Sending to " + url);

    let response = await sendRequestToServer(url);

    console.log(response);

    //update servo2 value
    document.querySelector("#servo2-value").innerHTML = servo2Slider.value;
}

// Add event on servo3 slider change
servo3Slider.addEventListener('input', updateServo3Value);

async function updateServo3Value(e) {
    let url = '/servo3/change' + '?value=' + servo3Slider.value;
    console.log("Sending to " + url);

    let response = await sendRequestToServer(url);

    console.log(response);

    //update servo3 value
    document.querySelector("#servo3-value").innerHTML = servo3Slider.value;
}


// Add event on servo4 slider change
servo4Slider.addEventListener('input', updateServo4Value);

async function updateServo4Value(e) {
    let url = '/servo4/change' + '?value=' + servo4Slider.value;
    console.log("Sending to " + url);

    let response = await sendRequestToServer(url);

    console.log(response);

    //update servo4 value
    document.querySelector("#servo4-value").innerHTML = servo4Slider.value;
}


