using System.Linq;

using Android.Content.PM;

namespace Demo.Droid.Helpers
{

    internal static class PermissionHelper
    {

        public static bool VerifyPermissions(Permission[] grantResults)
        {
            // At least one result must be checked.
            if (grantResults.Length < 1)
                return false;

            // Verify that each required permission has been granted, otherwise return false.
            return grantResults.All(result => result == Permission.Granted);

        }

    }

}