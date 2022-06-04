﻿// ------------------------------------------------------------------------------
//  Copyright (c) 2015 Microsoft Corporation
// 
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
// 
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
// 
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//  THE SOFTWARE.
// ------------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace Microsoft.OneDrive.Sdk.WindowsForms
{
    public class FormsWebAuthenticationUi : IWebAuthenticationUi
    {
        /// <summary>
        /// Displays authentication UI to the user for the specified request URI, returning
        /// the key value pairs from the query string upon reaching the callback URL.
        /// </summary>
        /// <param name="requestUri">The request URI.</param>
        /// <param name="callbackUri">The callback URI.</param>
        /// <returns>The <see cref="IDictionary{string, string}"/> of key value pairs from the callback URI query string.</returns>
        public async Task<IDictionary<string, string>> AuthenticateAsync(Uri requestUri, Uri callbackUri)
        {
            using (var formsDialog = new FormsWebDialog())
            {
                var responseValues = await formsDialog.GetAuthenticationResponseValues(requestUri, callbackUri);
                return responseValues;
            }
        }
    }
}
