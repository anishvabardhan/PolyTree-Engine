#include "JobSystem.hpp"

#include "Engine/Core/EngineCommon.hpp"

JobWorkerThread::JobWorkerThread(JobSystem* owner, unsigned int id)
{
	m_owner = owner;
	m_ID = id;

	m_workerThread = new std::thread(&JobWorkerThread::ThreadMain, this);
}

JobWorkerThread::~JobWorkerThread()
{
}

void JobWorkerThread::ThreadMain()
{
	while (!m_owner->m_isQuitting)
	{
		Job* claimedJob = m_owner->WorkerClaimAQueuedJob(this);

		if (claimedJob)
		{
			claimedJob->Execute();
			m_owner->WorkerCompleteAJob(this, claimedJob);
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::microseconds(1));
		}
	}
}

JobSystem::JobSystem(JobSystemConfig const& config)
{
	m_config = config;

	for (int workerIndex = 0; workerIndex < m_config.m_numOfWorkerThreads; workerIndex++)
	{
		JobWorkerThread* workerThread = new JobWorkerThread(this, workerIndex + 1);
		
		m_workerThreads.push_back(workerThread);
	}
}

JobSystem::~JobSystem()
{
	for (int workerIndex = 0; m_config.m_numOfWorkerThreads; workerIndex++)
	{
		DELETE_PTR(m_workerThreads[workerIndex]);
	}

	m_workerThreads.clear();
}

void JobSystem::StartUp()
{
}

void JobSystem::ShutDown()
{
	m_isQuitting = true;
}

void JobSystem::AddJob(Job* jobToAdd)
{
	jobToAdd->m_status = JobStatus::QUEUED;
	m_queuedJobsMutex.lock();
	m_queuedJobs.push_back(jobToAdd);
	m_queuedJobsMutex.unlock();
}

size_t JobSystem::GetNumOfQueuedJobs()
{
	return m_queuedJobs.size();
}

Job* JobSystem::WorkerClaimAQueuedJob(JobWorkerThread* workerThread)
{
	UNUSED(workerThread);

	m_queuedJobsMutex.lock();
	if (!m_queuedJobs.empty())
	{
		Job* claimedJob = m_queuedJobs.front();
		m_queuedJobs.pop_front();
		m_claimedJobsMutex.lock();
		claimedJob->m_status = JobStatus::EXECUTING;
		m_claimedJobs.push_back(claimedJob);
		m_claimedJobsMutex.unlock();
		m_queuedJobsMutex.unlock();

		return claimedJob;
	}

	m_queuedJobsMutex.unlock();
	return nullptr;
}

void JobSystem::WorkerCompleteAJob(JobWorkerThread* workerThread, Job* job)
{
	UNUSED(workerThread);

	m_claimedJobsMutex.lock();

	for (auto jobIter = m_claimedJobs.begin(); jobIter != m_claimedJobs.end(); jobIter++)
	{
		if (*jobIter == job)
		{
			m_claimedJobs.erase(jobIter);
			m_completedJobsMutex.lock();
			job->m_status = JobStatus::COMPLETED;
			m_completedJobs.push_back(job);
			m_completedJobsMutex.unlock();
			m_claimedJobsMutex.unlock();

			return;
		}
	}

	m_claimedJobsMutex.unlock();
}

bool JobSystem::RetrieveJob(Job* jobToRetrieve)
{
	m_completedJobsMutex.lock();

	for (auto jobIter = m_completedJobs.begin(); jobIter != m_completedJobs.end(); jobIter++)
	{
		if (*jobIter == jobToRetrieve)
		{
			m_completedJobs.erase(jobIter);
			jobToRetrieve->m_status = JobStatus::RETIEVED;
			m_completedJobsMutex.unlock();

			return true;
		}
	}

	m_completedJobsMutex.unlock();

	return false;
}

Job* JobSystem::RetrieveJob()
{
	std::lock_guard<std::mutex> lock(m_completedJobsMutex);

	if(m_completedJobs.empty())
		return nullptr;

	Job* job = m_completedJobs.front();
	m_completedJobs.pop_front();
	job->m_status = JobStatus::RETIEVED;

	return job;
}

void JobSystem::BeginFrame()
{
}

void JobSystem::EndFrame()
{
}
