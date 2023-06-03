import './App.css';
import data from "./data/countries.json";
import Countries from './Countries';

function App() {
  return (
    <Countries countries={data} />
  );
}

export default App;
