using System.Collections.Generic;
using System.Linq;
using Demo.Models;
using Prism.Windows.Mvvm;

namespace Demo.ViewModels
{

    internal sealed class MainPageViewModel : ViewModelBase
    {

        #region Events
        #endregion

        #region Fields
        #endregion

        #region Constructors

        public MainPageViewModel()
        {
            var items = new[]
            {
                new { Code="AF", Name="Afghanistan" },
                new { Code="AL", Name="Albania" },
                new { Code="DZ", Name="Algeria" },
                new { Code="AS", Name="American Samoa" },
                new { Code="AD", Name="Andorra" },
                new { Code="AO", Name="Angola" },
                new { Code="AI", Name="Anguilla" },
                new { Code="AQ", Name="Antarctica" },
                new { Code="AG", Name="Antigua and Barbuda" },
                new { Code="AR", Name="Argentina" },
                new { Code="AM", Name="Armenia" },
                new { Code="AW", Name="Aruba" },
                new { Code="AU", Name="Australia" },
                new { Code="AT", Name="Austria" },
                new { Code="AZ", Name="Azerbaijan" },
                new { Code="BS", Name="Bahamas (the)" },
                new { Code="BH", Name="Bahrain" },
                new { Code="BD", Name="Bangladesh" },
                new { Code="BB", Name="Barbados" },
                new { Code="BY", Name="Belarus" },
                new { Code="BE", Name="Belgium" },
                new { Code="BZ", Name="Belize" },
                new { Code="BJ", Name="Benin" },
                new { Code="BM", Name="Bermuda" },
                new { Code="BT", Name="Bhutan" },
                new { Code="BO", Name="Bolivia (Plurinational State of)" },
                new { Code="BQ", Name="Bonaire, Sint Eustatius and Saba" },
                new { Code="BA", Name="Bosnia and Herzegovina" },
                new { Code="BW", Name="Botswana" },
                new { Code="BV", Name="Bouvet Island" },
                new { Code="BR", Name="Brazil" },
                new { Code="IO", Name="British Indian Ocean Territory (the)" },
                new { Code="BN", Name="Brunei Darussalam" },
                new { Code="BG", Name="Bulgaria" },
                new { Code="BF", Name="Burkina Faso" },
                new { Code="BI", Name="Burundi" },
                new { Code="CV", Name="Cabo Verde" },
                new { Code="KH", Name="Cambodia" },
                new { Code="CM", Name="Cameroon" },
                new { Code="CA", Name="Canada" },
                new { Code="KY", Name="Cayman Islands (the)" },
                new { Code="CF", Name="Central African Republic (the)" },
                new { Code="TD", Name="Chad" },
                new { Code="CL", Name="Chile" },
                new { Code="CN", Name="China" },
                new { Code="CX", Name="Christmas Island" },
                new { Code="CC", Name="Cocos (Keeling) Islands (the)" },
                new { Code="CO", Name="Colombia" },
                new { Code="KM", Name="Comoros (the)" },
                new { Code="CD", Name="Congo (the Democratic Republic of the)" },
                new { Code="CG", Name="Congo (the)" },
                new { Code="CK", Name="Cook Islands (the)" },
                new { Code="CR", Name="Costa Rica" },
                new { Code="HR", Name="Croatia" },
                new { Code="CU", Name="Cuba" },
                new { Code="CW", Name="Curaçao" },
                new { Code="CY", Name="Cyprus" },
                new { Code="CZ", Name="Czechia" },
                new { Code="CI", Name="Côte d'Ivoire" },
                new { Code="DK", Name="Denmark" },
                new { Code="DJ", Name="Djibouti" },
                new { Code="DM", Name="Dominica" },
                new { Code="DO", Name="Dominican Republic (the)" },
                new { Code="EC", Name="Ecuador" },
                new { Code="EG", Name="Egypt" },
                new { Code="SV", Name="El Salvador" },
                new { Code="GQ", Name="Equatorial Guinea" },
                new { Code="ER", Name="Eritrea" },
                new { Code="EE", Name="Estonia" },
                new { Code="SZ", Name="Eswatini" },
                new { Code="ET", Name="Ethiopia" },
                new { Code="FK", Name="Falkland Islands (the) [Malvinas]" },
                new { Code="FO", Name="Faroe Islands (the)" },
                new { Code="FJ", Name="Fiji" },
                new { Code="FI", Name="Finland" },
                new { Code="FR", Name="France" },
                new { Code="GF", Name="French Guiana" },
                new { Code="PF", Name="French Polynesia" },
                new { Code="TF", Name="French Southern Territories (the)" },
                new { Code="GA", Name="Gabon" },
                new { Code="GM", Name="Gambia (the)" },
                new { Code="GE", Name="Georgia" },
                new { Code="DE", Name="Germany" },
                new { Code="GH", Name="Ghana" },
                new { Code="GI", Name="Gibraltar" },
                new { Code="GR", Name="Greece" },
                new { Code="GL", Name="Greenland" },
                new { Code="GD", Name="Grenada" },
                new { Code="GP", Name="Guadeloupe" },
                new { Code="GU", Name="Guam" },
                new { Code="GT", Name="Guatemala" },
                new { Code="GG", Name="Guernsey" },
                new { Code="GN", Name="Guinea" },
                new { Code="GW", Name="Guinea-Bissau" },
                new { Code="GY", Name="Guyana" },
                new { Code="HT", Name="Haiti" },
                new { Code="HM", Name="Heard Island and McDonald Islands" },
                new { Code="VA", Name="Holy See (the)" },
                new { Code="HN", Name="Honduras" },
                new { Code="HK", Name="Hong Kong" },
                new { Code="HU", Name="Hungary" },
                new { Code="IS", Name="Iceland" },
                new { Code="IN", Name="India" },
                new { Code="ID", Name="Indonesia" },
                new { Code="IR", Name="Iran (Islamic Republic of)" },
                new { Code="IQ", Name="Iraq" },
                new { Code="IE", Name="Ireland" },
                new { Code="IM", Name="Isle of Man" },
                new { Code="IL", Name="Israel" },
                new { Code="IT", Name="Italy" },
                new { Code="JM", Name="Jamaica" },
                new { Code="JP", Name="Japan" },
                new { Code="JE", Name="Jersey" },
                new { Code="JO", Name="Jordan" },
                new { Code="KZ", Name="Kazakhstan" },
                new { Code="KE", Name="Kenya" },
                new { Code="KI", Name="Kiribati" },
                new { Code="KP", Name="Korea (the Democratic People's Republic of)" },
                new { Code="KR", Name="Korea (the Republic of)" },
                new { Code="KW", Name="Kuwait" },
                new { Code="KG", Name="Kyrgyzstan" },
                new { Code="LA", Name="Lao People's Democratic Republic (the)" },
                new { Code="LV", Name="Latvia" },
                new { Code="LB", Name="Lebanon" },
                new { Code="LS", Name="Lesotho" },
                new { Code="LR", Name="Liberia" },
                new { Code="LY", Name="Libya" },
                new { Code="LI", Name="Liechtenstein" },
                new { Code="LT", Name="Lithuania" },
                new { Code="LU", Name="Luxembourg" },
                new { Code="MO", Name="Macao" },
                new { Code="MG", Name="Madagascar" },
                new { Code="MW", Name="Malawi" },
                new { Code="MY", Name="Malaysia" },
                new { Code="MV", Name="Maldives" },
                new { Code="ML", Name="Mali" },
                new { Code="MT", Name="Malta" },
                new { Code="MH", Name="Marshall Islands (the)" },
                new { Code="MQ", Name="Martinique" },
                new { Code="MR", Name="Mauritania" },
                new { Code="MU", Name="Mauritius" },
                new { Code="YT", Name="Mayotte" },
                new { Code="MX", Name="Mexico" },
                new { Code="FM", Name="Micronesia (Federated States of)" },
                new { Code="MD", Name="Moldova (the Republic of)" },
                new { Code="MC", Name="Monaco" },
                new { Code="MN", Name="Mongolia" },
                new { Code="ME", Name="Montenegro" },
                new { Code="MS", Name="Montserrat" },
                new { Code="MA", Name="Morocco" },
                new { Code="MZ", Name="Mozambique" },
                new { Code="MM", Name="Myanmar" },
                new { Code="NA", Name="Namibia" },
                new { Code="NR", Name="Nauru" },
                new { Code="NP", Name="Nepal" },
                new { Code="NL", Name="Netherlands (Kingdom of the)" },
                new { Code="NC", Name="New Caledonia" },
                new { Code="NZ", Name="New Zealand" },
                new { Code="NI", Name="Nicaragua" },
                new { Code="NE", Name="Niger (the)" },
                new { Code="NG", Name="Nigeria" },
                new { Code="NU", Name="Niue" },
                new { Code="NF", Name="Norfolk Island" },
                new { Code="MK", Name="North Macedonia" },
                new { Code="MP", Name="Northern Mariana Islands (the)" },
                new { Code="NO", Name="Norway" },
                new { Code="OM", Name="Oman" },
                new { Code="PK", Name="Pakistan" },
                new { Code="PW", Name="Palau" },
                new { Code="PS", Name="Palestine, State of" },
                new { Code="PA", Name="Panama" },
                new { Code="PG", Name="Papua New Guinea" },
                new { Code="PY", Name="Paraguay" },
                new { Code="PE", Name="Peru" },
                new { Code="PH", Name="Philippines (the)" },
                new { Code="PN", Name="Pitcairn" },
                new { Code="PL", Name="Poland" },
                new { Code="PT", Name="Portugal" },
                new { Code="PR", Name="Puerto Rico" },
                new { Code="QA", Name="Qatar" },
                new { Code="RO", Name="Romania" },
                new { Code="RU", Name="Russian Federation (the)" },
                new { Code="RW", Name="Rwanda" },
                new { Code="RE", Name="Réunion" },
                new { Code="BL", Name="Saint Barthélemy" },
                new { Code="SH", Name="Saint Helena, Ascension and Tristan da Cunha" },
                new { Code="KN", Name="Saint Kitts and Nevis" },
                new { Code="LC", Name="Saint Lucia" },
                new { Code="MF", Name="Saint Martin (French part)" },
                new { Code="PM", Name="Saint Pierre and Miquelon" },
                new { Code="VC", Name="Saint Vincent and the Grenadines" },
                new { Code="WS", Name="Samoa" },
                new { Code="SM", Name="San Marino" },
                new { Code="ST", Name="Sao Tome and Principe" },
                new { Code="SA", Name="Saudi Arabia" },
                new { Code="SN", Name="Senegal" },
                new { Code="RS", Name="Serbia" },
                new { Code="SC", Name="Seychelles" },
                new { Code="SL", Name="Sierra Leone" },
                new { Code="SG", Name="Singapore" },
                new { Code="SX", Name="Sint Maarten (Dutch part)" },
                new { Code="SK", Name="Slovakia" },
                new { Code="SI", Name="Slovenia" },
                new { Code="SB", Name="Solomon Islands" },
                new { Code="SO", Name="Somalia" },
                new { Code="ZA", Name="South Africa" },
                new { Code="GS", Name="South Georgia and the South Sandwich Islands" },
                new { Code="SS", Name="South Sudan" },
                new { Code="ES", Name="Spain" },
                new { Code="LK", Name="Sri Lanka" },
                new { Code="SD", Name="Sudan (the)" },
                new { Code="SR", Name="Suriname" },
                new { Code="SJ", Name="Svalbard and Jan Mayen" },
                new { Code="SE", Name="Sweden" },
                new { Code="CH", Name="Switzerland" },
                new { Code="SY", Name="Syrian Arab Republic (the)" },
                new { Code="TW", Name="Taiwan (Province of China)" },
                new { Code="TJ", Name="Tajikistan" },
                new { Code="TZ", Name="Tanzania, the United Republic of" },
                new { Code="TH", Name="Thailand" },
                new { Code="TL", Name="Timor-Leste" },
                new { Code="TG", Name="Togo" },
                new { Code="TK", Name="Tokelau" },
                new { Code="TO", Name="Tonga" },
                new { Code="TT", Name="Trinidad and Tobago" },
                new { Code="TN", Name="Tunisia" },
                new { Code="TM", Name="Turkmenistan" },
                new { Code="TC", Name="Turks and Caicos Islands (the)" },
                new { Code="TV", Name="Tuvalu" },
                new { Code="TR", Name="Türkiye" },
                new { Code="UG", Name="Uganda" },
                new { Code="UA", Name="Ukraine" },
                new { Code="AE", Name="United Arab Emirates (the)" },
                new { Code="GB", Name="United Kingdom of Great Britain and Northern Ireland (the)" },
                new { Code="UM", Name="United States Minor Outlying Islands (the)" },
                new { Code="US", Name="United States of America (the)" },
                new { Code="UY", Name="Uruguay" },
                new { Code="UZ", Name="Uzbekistan" },
                new { Code="VU", Name="Vanuatu" },
                new { Code="VE", Name="Venezuela (Bolivarian Republic of)" },
                new { Code="VN", Name="Viet Nam" },
                new { Code="VG", Name="Virgin Islands (British)" },
                new { Code="VI", Name="Virgin Islands (U.S.)" },
                new { Code="WF", Name="Wallis and Futuna" },
                new { Code="EH", Name="Western Sahara*" },
                new { Code="YE", Name="Yemen" },
                new { Code="ZM", Name="Zambia" },
                new { Code="ZW", Name="Zimbabwe" },
                new { Code="AX", Name="Åland Islands" }
            };

            this.Sources = items.Select(arg => arg.Name).ToArray();
            this.Sources2 = items.Select(arg => new KeyValuePair<string, string>(arg.Code, arg.Name)).ToArray();
            this.Sources3 = items.Select(arg => new ItemModel(arg.Code, arg.Name)).ToArray();
        }

        #endregion

        #region Properties

        private IReadOnlyCollection<string> _Sources;

        public IReadOnlyCollection<string> Sources
        {
            get => this._Sources;
            set => this.SetProperty(ref this._Sources, value);
        }

        private IReadOnlyCollection<KeyValuePair<string, string>> _Sources2;

        public IReadOnlyCollection<KeyValuePair<string, string>> Sources2
        {
            get => this._Sources2;
            set => this.SetProperty(ref this._Sources2, value);
        }

        private IReadOnlyCollection<ItemModel> _Sources3;

        public IReadOnlyCollection<ItemModel> Sources3
        {
            get => this._Sources3;
            set => this.SetProperty(ref this._Sources3, value);
        }

        #endregion

        #region Methods

        #region Overrides
        #endregion

        #region Event Handlers
        #endregion

        #region Helpers
        #endregion

        #endregion

    }

}
