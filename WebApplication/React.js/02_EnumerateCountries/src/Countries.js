import Country from './Country';

export default function Countries({ countries }) {
  return (
    <ul className='countries'>
      {countries.map((country, i) => (
        <Country key={i} {...country} />
      ))}
    </ul>
  );
}
