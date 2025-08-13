//
//  ContentView.swift
//  Demo
//
//  Created by Takuya Takeuchi on 2024/02/23.
//

import SwiftUI

struct ContentView: View {
    @State private var text = ""
    
    var body: some View {
        VStack {
            TextField("Card number", text: self.$text).textContentType(.creditCardNumber)
        }
        .padding()
    }
}

#Preview {
    ContentView()
}
