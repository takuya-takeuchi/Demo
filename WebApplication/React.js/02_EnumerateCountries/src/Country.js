import './App.css';

export default function Country({ name, capital }) {
  return (
    <li>
      {name} {capital}
    </li>
  );
}
