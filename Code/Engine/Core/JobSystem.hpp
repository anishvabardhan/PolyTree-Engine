#pragma once

#include <deque>
#include <mutex>
#include <thread>
#include <vector>

class JobSystem;

enum class JobStatus
{
	NO_RECORD = -1,
	QUEUED,
	EXECUTING,
	COMPLETED,
	RETIEVED
};

class Job
{
public:
	std::atomic<JobStatus> m_status = JobStatus::NO_RECORD;
public:
	Job() = default;
	virtual ~Job() = default;

	virtual void Execute() = 0;
};

class JobWorkerThread
{
	unsigned int m_ID = 0;
	JobSystem* m_owner = nullptr;
	std::thread* m_workerThread;
public:
	JobWorkerThread(JobSystem* owner, unsigned int id);
	~JobWorkerThread();

	void ThreadMain();
private:
	friend class JobSystem;
};

struct JobSystemConfig
{
	int m_numOfWorkerThreads = 0;
};

class JobSystem
{
	JobSystemConfig m_config;

	std::mutex m_queuedJobsMutex;
	std::mutex m_claimedJobsMutex;
	std::mutex m_completedJobsMutex;

	std::vector<JobWorkerThread*> m_workerThreads;

	std::deque<Job*> m_queuedJobs;
	std::deque<Job*> m_claimedJobs;
	std::deque<Job*> m_completedJobs;
protected:
	std::atomic<bool> m_isQuitting = false;
public:
	JobSystem(JobSystemConfig const& config);
	~JobSystem();

	void StartUp();
	void BeginFrame();
	void EndFrame();
	void ShutDown();

	void AddJob(Job* jobToAdd);

	size_t GetNumOfQueuedJobs();

	Job* WorkerClaimAQueuedJob(JobWorkerThread* workerThread);
	void WorkerCompleteAJob(JobWorkerThread* workerThread, Job* job);
	bool RetrieveJob(Job* jobToRetrieve);
	Job* RetrieveJob();
private:
	friend class JobWorkerThread;
};