// winid: list on-screen windows of an app: "<id> <width> <height> <title>" (for screencapture -l)
import CoreGraphics
let owner = CommandLine.arguments[1]
let list = CGWindowListCopyWindowInfo([.optionOnScreenOnly], kCGNullWindowID) as! [[String: Any]]
for w in list where (w[kCGWindowOwnerName as String] as? String) == owner {
    let b = w[kCGWindowBounds as String] as! [String: Any]
    print(w[kCGWindowNumber as String]!, b["Width"]!, b["Height"]!, w[kCGWindowName as String] ?? "")
}
