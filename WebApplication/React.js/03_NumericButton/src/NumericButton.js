import './NumericButton.css';
import { useState } from 'react';

export default function NumericButton() {
  const [value, setValue] = useState(0);
  return (
    <div id="numericButton">
      <div id="numericButton-value">
        <p>{value}</p>
      </div>
      <button id="numericButton-upper" onClick={() => setValue(value + 1)}>
        &uarr;
      </button>
      <button id="numericButton-down" onClick={() => setValue(value - 1)}>
        &darr;
      </button>
    </div>
  );
}
