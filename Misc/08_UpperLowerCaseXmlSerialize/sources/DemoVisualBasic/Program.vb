Imports System.IO
Imports System.Xml.Serialization

Imports NLog

Module Program

#Region "Fields"

    Private ReadOnly Logger = LogManager.GetCurrentClassLogger()

#End Region

#Region "Methods"

    Sub Main(args As String())
        TestDeserialize(Of Test)("TestUpper.xml")
        TestDeserialize(Of Test)("TestLower.xml")
    End Sub

#End Region

#Region "Helpers"

    Sub TestDeserialize(Of T)(filename As String)
        Try
            Dim serializer = New XmlSerializer(GetType(T))
            Using fileStream = New FileStream(filename, FileMode.Open, FileAccess.Read)
                serializer.Deserialize(fileStream)
            End Using
            Logger.Info($"Succeed to deserialize {GetType(T).FullName} from {filename}")
        Catch ex As Exception
            Logger.Error($"Failed to deserialize {GetType(T).FullName} from {filename}")
        End Try
    End Sub

#End Region

End Module

Public Class Test
    Public Property Member As Integer
End Class

'Can not declare
'Public Class TEST
'    Public Property Member As Integer
'End Class
