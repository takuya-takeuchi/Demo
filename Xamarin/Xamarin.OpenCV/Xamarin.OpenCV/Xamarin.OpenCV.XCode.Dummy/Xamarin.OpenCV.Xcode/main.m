//
//  main.m
//  Xamarin.OpenCV.Xcode
//
//  Created by Takuya Takeuchi on 2020/05/29.
//  Copyright © 2020 Takuya Takeuchi. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "AppDelegate.h"

int main(int argc, char * argv[]) {
    NSString * appDelegateClassName;
    @autoreleasepool {
        // Setup code that might create autoreleased objects goes here.
        appDelegateClassName = NSStringFromClass([AppDelegate class]);
    }
    return UIApplicationMain(argc, argv, nil, appDelegateClassName);
}
