#***************************************
#Arguments
#%1: Host of GitLab (e.g. https://www.contoso.com/gitlab)
#%2: Token of API
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $GitLabHost,

   [Parameter(
   Mandatory=$True,
   Position = 2
   )][string]
   $Token
)

$page = 1
$per_page = 20
$porject_lists = @();
while ($True)
{
    $projects = (Invoke-WebRequest -Headers @{"Content-type"="application/json"} `
                                   "${GitLabHost}/api/v4/projects?simple=true&private_token=${Token}&page=${page}&per_page=${per_page}" | ConvertFrom-Json)
    if ($projects.Count -eq 0)
    {
        break;
    }

    foreach ($project in $projects)
    {
        $porject_lists += $project.id
    }

    $page += 1
}

$count = $porject_lists.Length
Write-Host "Total Projects: ${count}" -ForegroundColor Blue

# pipelines
foreach ($project_id in $porject_lists)
{
    $page = 1
    $pipleline_lists = @();
    $pipleline_finished_lists = @();
    while ($True)
    {
        $piplelines = (Invoke-WebRequest -Headers @{"Content-type"="application/json"} `
                                         "${GitLabHost}/api/v4/projects/${project_id}/pipelines?private_token=${Token}&page=${page}&per_page=${per_page}" | ConvertFrom-Json)
        $count = $piplelines.Count
        if ($count -eq 0)
        {
            break;
        }

        if ($count -eq 1)
        {
            $message = $piplelines[0].message
            if ($message)
            {
                Write-Host "Failed to get pipelines from Project '${project_id}'. Reason: ${message}" -ForegroundColor Red
                break;
            }
        }

        foreach ($pipleline in $piplelines)
        {
            $pipleline_lists += $pipleline.id
            if (($pipleline.status -eq "success") -or
                ($pipleline.status -eq "failed") -or
                ($pipleline.status -eq "canceled") -or
                ($pipleline.status -eq "skipped"))
            {
                $pipleline_finished_lists += $pipleline.id
            }
        }

        $page += 1
    }

    $count = $pipleline_lists.Count
    Write-Host "Total Pipelines: ${count} in Project '${project_id}'" -ForegroundColor Blue
    if ($count -ne 0)
    {
        foreach ($pipleline_id in $pipleline_finished_lists)
        {
            $result = (Invoke-WebRequest -Headers @{"Content-type"="application/json"} `
                                         -Method Delete `
                                         "${GitLabHost}/api/v4/projects/${project_id}/pipelines/${pipleline_id}?private_token=${Token}" | ConvertFrom-Json)
            $message = $result.message
            if ($message)
            {
                Write-Host "Failed to delete '${pipleline_id}' in Project '${project_id}'. Reason: ${message}" -ForegroundColor Red
            }
            else
            {
                Write-Host "Success to delete '${pipleline_id}' in Project '${project_id}'" -ForegroundColor Green
            }
        }
    }
}
